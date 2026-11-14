#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>
#include <cerrno>
#include <Eigen/Core>
#include <functional>
#include <random>
#include <climits>
#include <cmath>
#include <limits>
#include <memory>
#include <algorithm>
// Offline OpenGV evaluation on the same cached frame pairs. No ROS master needed.
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/TranslationOnlySacProblem.hpp>
#include <opengv/sac_problems/point_cloud/PointCloudSacProblem.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/point_cloud/PointCloudAdapter.hpp>
#include <Eigen/SVD>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

unsigned int evaluation_seed = 12345u;

using Central = opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;
using Translation = opengv::sac_problems::relative_pose::TranslationOnlySacProblem;
using Cloud = opengv::sac_problems::point_cloud::PointCloudSacProblem;
using Adapter = opengv::relative_pose::CentralRelativeAdapter;
using Adapter3D = opengv::point_cloud::PointCloudAdapter;

std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> cells;
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ',')) cells.push_back(cell);
  return cells;
}

template<class Problem>
bool runRansac(const std::shared_ptr<Problem>& problem, double threshold,
               bool refine, opengv::transformation_t& model,
               std::vector<int>& inliers, size_t n) {
  if (n < static_cast<size_t>(problem->getSampleSize())) return false;
  // OpenGV stores a COPY of the RNG inside std::bind, so rebuild it after seeding.
  problem->rng_alg_.seed(evaluation_seed);
  problem->rng_gen_.reset(new std::function<int()>(
      std::bind(*problem->rng_dist_, problem->rng_alg_)));
  opengv::sac::Ransac<Problem> ransac;
  ransac.sac_model_ = problem;
  ransac.threshold_ = threshold;
  ransac.max_iterations_ = 1000;
  if (!ransac.computeModel() || !ransac.model_coefficients_.allFinite()) return false;
  model = ransac.model_coefficients_;
  inliers = ransac.inliers_;
  if (inliers.size() < 3) return false;
  if (refine) {
    opengv::transformation_t optimized = model;
    problem->optimizeModelCoefficients(inliers, model, optimized);
    if (!optimized.allFinite()) return false;
    model = optimized;
  }
  return model.allFinite();
}

int main(int argc, char** argv) {
  if (argc < 3 || argc > 6) {
    std::cerr << "Usage: cache_eval CACHE_PREFIX NEW_OUTPUT_PREFIX [bearing_threshold=1e-6] [seed=12345] [timeout_seconds=2]\n";
    return 2;
  }
  try {
    const std::string input = argv[1], output = argv[2];
    const double threshold = argc >= 4 ? std::stod(argv[3]) : 1e-6;
    if (argc >= 5) {
      const unsigned long parsed = std::stoul(argv[4]);
      if (parsed > std::numeric_limits<unsigned int>::max())
        throw std::runtime_error("Seed out of range");
      evaluation_seed = static_cast<unsigned int>(parsed);
    }
    const double timeout_seconds = argc >= 6 ? std::stod(argv[5]) : 2.0;
    if (!std::isfinite(timeout_seconds) || timeout_seconds <= 0)
      throw std::runtime_error("Invalid solver timeout");
    if (!std::isfinite(threshold) || threshold <= 0) throw std::runtime_error("Invalid threshold");
    std::ifstream frames(input + "_frames.csv"), matches(input + "_matches.csv");
    if (!frames || !matches) throw std::runtime_error("Cannot open input cache");
    const std::string names[] = {"nister", "eightpt", "twopt", "arun"};
    // Preflight all paths before creating any output.
    for (const auto& name : names) {
      if (std::ifstream(output + "_" + name + ".csv").good())
        throw std::runtime_error("Output already exists; choose a NEW output prefix");
    }
    std::ofstream out[4];
    for (int m = 0; m < 4; ++m) {
      out[m].open(output + "_" + names[m] + ".csv");
      if (!out[m]) throw std::runtime_error("Cannot create output");
      out[m] << "image_time_s,method,rotation_frobenius,translation_error,translation_unit,"
                "frame_id,previous_image_time_s,status,correspondences,inliers,solver_ms\n"
             << std::setprecision(17);
    }
    std::cout << "Shared bearing threshold=" << threshold
              << "; 5pt/8pt both refined; OpenGV seed=" << evaluation_seed
              << "; Arun threshold=0.05 m; per-method timeout="<<timeout_seconds<<" s"<<std::endl;
    std::string line;
    std::getline(frames, line);
    if (line.find("frame_id,previous_image_time_s,image_time_s,match_count") != 0)
      throw std::runtime_error("Wrong frames schema");
    std::getline(matches, line);
    if (line.find("frame_id,match_id,b1x,b1y,b1z") != 0)
      throw std::runtime_error("Wrong matches schema");
    size_t frame_count = 0, successes[4] = {0,0,0,0};
    double sums[4] = {0,0,0,0};
    while (std::getline(frames, line)) {
      if (line.empty()) continue;
      auto f = split(line);
      if (f.size() != 17) throw std::runtime_error("Malformed frame row");
      const size_t id = std::stoull(f[0]), n = std::stoull(f[3]);
      if (id != frame_count) throw std::runtime_error("Non-contiguous frame IDs");
      const double previous = std::stod(f[1]), stamp = std::stod(f[2]);
      if (!(stamp > previous)) throw std::runtime_error("Nonpositive frame interval");
      Eigen::Matrix3d gtR;
      Eigen::Vector3d gtT;
      for (int i=0;i<9;++i) gtR(i/3,i%3)=std::stod(f[5+i]);
      for (int i=0;i<3;++i) gtT(i)=std::stod(f[14+i]);
      if (!gtR.allFinite() || !gtT.allFinite()) throw std::runtime_error("Invalid GT");
      opengv::bearingVectors_t b1,b2;
      std::vector<double> depths1, depths2;
      b1.reserve(n); b2.reserve(n); depths1.reserve(n); depths2.reserve(n);
      for (size_t i=0;i<n;++i) {
        if (!std::getline(matches,line)) throw std::runtime_error("Truncated matches cache");
        auto r=split(line);
        if (r.size()!=10 || std::stoull(r[0])!=id || std::stoull(r[1])!=i)
          throw std::runtime_error("Match ID/count mismatch");
        opengv::bearingVector_t a,b;
        for(int j=0;j<3;++j) {a(j)=std::stod(r[2+j]);b(j)=std::stod(r[5+j]);}
        if(!a.allFinite() || !b.allFinite() || std::abs(a.norm()-1)>1e-6 || std::abs(b.norm()-1)>1e-6)
          throw std::runtime_error("Invalid unit bearings");
        b1.push_back(a);b2.push_back(b);
        depths1.push_back(std::stod(r[8]));depths2.push_back(std::stod(r[9]));
      }
      for (int method=0;method<4;++method) {
        const auto tic=std::chrono::steady_clock::now();
        opengv::transformation_t model;
        std::vector<int> inliers;
        bool valid=false;
        size_t used=n;
        struct SharedResult {
          double coefficients[12];
          size_t used, inliers;
          bool valid;
        };
        void* memory = mmap(nullptr, sizeof(SharedResult), PROT_READ | PROT_WRITE,
                            MAP_SHARED | MAP_ANONYMOUS, -1, 0);
        if (memory == MAP_FAILED) throw std::runtime_error("mmap failed");
        SharedResult* shared = static_cast<SharedResult*>(memory);
        shared->valid = false; shared->used = n; shared->inliers = 0;
        const pid_t child = fork();
        if (child < 0) {
          munmap(memory, sizeof(SharedResult));
          throw std::runtime_error("fork failed");
        }
        if (child == 0) {
          try {
        if (method<2) {
          Adapter adapter(b1,b2);
          auto problem=std::make_shared<Central>(adapter,method==0?Central::NISTER:Central::EIGHTPT,false);
          valid=runRansac(problem,threshold,true,model,inliers,n);
        } else if (method==2) {
          Adapter adapter(b1,b2);
          adapter.setR12(gtR);
          auto problem=std::make_shared<Translation>(adapter,false);
          valid=runRansac(problem,threshold,false,model,inliers,n);
          if (valid) {
            Eigen::MatrixXd A(inliers.size(),3);
            for(size_t k=0;k<inliers.size();++k)
              A.row(k)=b1[inliers[k]].cross(gtR*b2[inliers[k]]).transpose();
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(A,Eigen::ComputeFullV);
            if (svd.singularValues()(1)>1e-10) {
              Eigen::Vector3d t=svd.matrixV().col(2).normalized();
              if (t.dot(model.col(3))<0) t=-t;
              model.block<3,3>(0,0)=gtR;model.col(3)=t;
            }
          }
        } else {
          opengv::points_t p1,p2;
          for(size_t i=0;i<n;++i) {
            if(std::isfinite(depths1[i]) && std::isfinite(depths2[i]) &&
               depths1[i]>0 && depths2[i]>0 && std::abs(b1[i].z())>1e-8 && std::abs(b2[i].z())>1e-8) {
              p1.push_back(b1[i]*(depths1[i]/b1[i].z()));
              p2.push_back(b2[i]*(depths2[i]/b2[i].z()));
            }
          }
          used=p1.size();
          Adapter3D adapter(p1,p2);
          auto problem=std::make_shared<Cloud>(adapter,false);
          valid=runRansac(problem,0.05,true,model,inliers,used);
        }

            if (valid && model.allFinite()) {
              for (int j=0;j<12;++j) shared->coefficients[j] = model.data()[j];
              shared->valid = true;
            }
            shared->used = used; shared->inliers = inliers.size();
            _exit(0);  // Never flush inherited parent CSV streams in the child.
          } catch (...) {
            _exit(3);
          }
        }
        bool timed_out = false;
        int child_status = 0;
        while (true) {
          const pid_t state = waitpid(child, &child_status, WNOHANG);
          if (state == child) break;
          if (state < 0 && errno != EINTR) {
            kill(child, SIGKILL); waitpid(child, &child_status, 0);
            munmap(memory, sizeof(SharedResult));
            throw std::runtime_error("waitpid failed");
          }
          if (std::chrono::duration<double>(std::chrono::steady_clock::now()-tic).count()
              >= timeout_seconds) {
            kill(child, SIGKILL);
            while (waitpid(child, &child_status, 0) < 0 && errno == EINTR) {}
            timed_out = true;
            std::cerr << "TIMEOUT seed=" << evaluation_seed << " frame=" << id
                      << " time=" << std::setprecision(17) << stamp
                      << " method=" << names[method] << " limit=" << timeout_seconds
                      << " s" << std::endl;
            break;
          }
          usleep(500);
        }
        valid = !timed_out && WIFEXITED(child_status) &&
                WEXITSTATUS(child_status) == 0 && shared->valid;
        used = shared->used;
        const size_t reported_inliers = shared->inliers;
        if (valid) for (int j=0;j<12;++j) model.data()[j]=shared->coefficients[j];
        munmap(memory, sizeof(SharedResult));
        const double elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-tic).count();
        std::string status=timed_out ? "solver_timeout" : "solver_failed";
        double er=std::numeric_limits<double>::quiet_NaN(), et=er;
        if(valid && model.allFinite()) {
          er=(gtR-model.block<3,3>(0,0)).norm();
          const Eigen::Vector3d estimate=model.col(3);
          if(method<3 && (gtT.norm()<1e-8 || estimate.norm()<1e-8)) {
            status="zero_translation";
          } else {
            if(method<3) et=(gtT.normalized()-estimate.normalized()).norm();
            else et=(gtT-estimate).norm();
            status="ok"; ++successes[method];sums[method]+=et;
          }
        }
        out[method]<<stamp<<","<<method<<","<<er<<","<<et<<","<<(method<3?"direction":"m")
                   <<","<<id<<","<<previous<<","<<status<<","<<used<<","<<reported_inliers<<","<<elapsed<<"\n";
        if(!out[method].good())throw std::runtime_error("Output write failed");
      }
      ++frame_count;
      if(frame_count%100==0)std::cout<<"Processed "<<frame_count<<" shared frame pairs"<<std::endl;
      for (auto& stream : out) stream.flush();
    }
    if(frame_count==0)throw std::runtime_error("Empty cache");
    if(std::getline(matches,line) && !line.empty())std::cerr<<"WARNING: uncommitted trailing matches ignored\n";
    for(int m=0;m<4;++m) {
      out[m].flush();
      if(!out[m].good())throw std::runtime_error("Output flush failed");
      std::cout<<names[m]<<": ok="<<successes[m]<<"/"<<frame_count;
      if(successes[m])std::cout<<" translation mean="<<sums[m]/successes[m];
      std::cout<<"\n";
    }
    std::cout<<"DONE. Prefix: "<<output<<"\n";
  } catch(const std::exception& e) {
    std::cerr<<"ERROR: "<<e.what()<<"\n";
    return 1;
  }
  return 0;
}
