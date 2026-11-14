# -*- coding: utf-8 -*-
"""Regenerate figures from organized CSVs. Python + matplotlib; no ROS needed."""
from pathlib import Path
import csv
import statistics
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
RAW=ROOT/'results'/'seed_runs'
CURVES=ROOT/'figures'/'error_curves'
ROBUST=ROOT/'figures'/'robustness'
SEEDS=[12345,23456,34567,45678,56789]
METHODS=['nister','eightpt','twopt','arun']
LABELS=['5-point Nister','8-point','2-point (GT rotation)','Arun 3D-3D']
COLORS=['#2878B5','#E58B24','#29924C','#7760A5']
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,
                     'axes.titlesize':13,'axes.labelsize':11,
                     'axes.spines.top':False,'axes.spines.right':False})

def load(path):
    with path.open() as f:return list(csv.DictReader(f))

def save(fig,folder,name):
    fig.savefig(str(folder/(name+'.png')),dpi=220,bbox_inches='tight')
    fig.savefig(str(folder/(name+'.svg')),bbox_inches='tight')
    plt.close(fig)
    print('Saved:',name)

def curve(methods,key,ylabel,title,name):
    fig,ax=plt.subplots(figsize=(11,4.6))
    for m in methods:
        rows=load(RAW/('seed_12345_'+METHODS[m]+'.csv'))
        # Plot failed solves as gaps, rather than connecting through them.
        times=[float(r['image_time_s'])-24 for r in rows]
        values=[float(r[key]) if r['status']=='ok' else float('nan') for r in rows]
        ax.plot(times,values,lw=.65,color=COLORS[m],label=LABELS[m],alpha=.9)
    ax.set(xlabel='Time since bag start (s)',ylabel=ylabel,title=title,xlim=(0,237))
    ax.grid(alpha=.2);ax.legend(fontsize=9,loc='upper right')
    fig.text(.13,.015,'Shared cached frame pairs | predeclared representative seed: 12345 | raw errors, no smoothing',fontsize=8,color='#555555')
    fig.subplots_adjust(bottom=.18)
    save(fig,CURVES,name)

curve([0,1,2],'rotation_frobenius','Rotation error (Frobenius norm)',
      '2D-2D relative rotation error','2d_rotation_seed12345')
curve([0,1,2],'translation_error','Translation direction error (unitless)',
      '2D-2D relative translation direction error','2d_translation_seed12345')
curve([3],'rotation_frobenius','Rotation error (Frobenius norm)',
      '3D-3D relative rotation error','arun_rotation_seed12345')
curve([3],'translation_error','Translation error (m)',
      '3D-3D relative translation error','arun_translation_seed12345')

summary=load(ROOT/'results/tables/across_seed_summary.csv')
perrun=load(ROOT/'results/tables/per_run_metrics.csv')
fig,axes=plt.subplots(1,2,figsize=(11,4.8),gridspec_kw={'width_ratios':[3,1.3]})
for ax,indices in zip(axes,[range(3),[3]]):
    means=[float(summary[i]['translation_run_mean_average']) for i in indices]
    errors=[float(summary[i]['translation_run_mean_sample_std']) for i in indices]
    ticks=list(range(len(means)))
    ax.bar(ticks,means,yerr=errors,capsize=6,color=[COLORS[i] for i in indices],width=.6)
    ax.set_xticks(ticks);ax.set_xticklabels([LABELS[i] for i in indices],fontsize=9)
    ax.set_ylim(0,max(means)*1.3);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
    for x,mean,error in zip(ticks,means,errors):
        ax.text(x,mean+error+max(means)*.025,'%.5f\n+/- %.5f'%(mean,error),ha='center',fontsize=9)
axes[0].set_ylabel('Direction error (unitless)');axes[1].set_ylabel('Translation error (m)')
axes[0].set_title('2D translation');axes[1].set_title('Arun metric translation')
fig.suptitle('Across-seed average of full-run mean translation errors',fontsize=13,y=.99)
fig.text(.08,.025,'Error bars: SAMPLE standard deviation across 5 prespecified seeds. Errors conditional on successful solves.',fontsize=8)
fig.subplots_adjust(wspace=.4,bottom=.19,top=.83)
save(fig,ROBUST,'across_seed_mean_std')

fig,axes=plt.subplots(1,2,figsize=(11,4.7),gridspec_kw={'width_ratios':[2.4,1.6]})
for i,m in enumerate(METHODS):
    rows={int(r['seed']):r for r in perrun if r['method']==m}
    vals=[float(rows[seed]['translation_mean']) for seed in SEEDS]
    ax=axes[0] if i<3 else axes[1]
    ax.plot(range(5),vals,'o-',lw=1.4,color=COLORS[i],label=LABELS[i])
for ax in axes:
    ax.set_xticks(range(5));ax.set_xticklabels([str(s) for s in SEEDS],rotation=25)
    ax.set_xlabel('Prespecified random seed');ax.grid(alpha=.2)
axes[0].set(ylabel='Mean direction error (unitless)',ylim=(0,.6),title='2D methods: each run')
axes[1].set(ylabel='Mean translation error (m)',title='Arun: each run',ylim=(.0038,.0043))
axes[0].legend(fontsize=8);axes[1].legend(fontsize=8)
fig.text(.08,.015,'Same 3,889 cached pairs per seed; failed estimates excluded from error means and counted separately.',fontsize=8)
fig.subplots_adjust(wspace=.4,bottom=.25)
save(fig,ROBUST,'per_seed_errors')

fig,ax=plt.subplots(figsize=(9,4.4))
fail=[int(r['failed_total']) for r in summary]
ax.bar(range(4),fail,color=COLORS,width=.55)
ax.set_xticks(range(4));ax.set_xticklabels(LABELS,fontsize=9)
ax.set(ylabel='Failed solves across 5 seeds',title='Failure accounting: 19,445 attempts per method',ylim=(0,5.2))
ax.set_yticks(range(6));ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
for i,n in enumerate(fail):
    ax.text(i,n+.12,'%d / 19,445\n(%.4f%%)'%(n,100*n/19445),ha='center',fontsize=10)
fig.text(.08,.015,'All 4 failures are Nister timeouts; 2 s process watchdog. Seed 12345 reused a completed non-isolated run.',fontsize=8)
fig.subplots_adjust(bottom=.19)
save(fig,ROBUST,'failure_counts')

table_labels=['Five-point','Eight-point','Two-point (GT rotation)','Arun 3D alignment']
with (ROOT/'results/tables/summary.csv').open('w',newline='',encoding='utf-8') as f:
    writer=csv.writer(f)
    writer.writerow(['method','runs','translation_unit','average_of_run_means','sample_std_of_run_means','min_run_mean','max_run_mean','failed_solves','timeout_solves','attempts'])
    for label,r in zip(table_labels,summary):
        writer.writerow([label,r['runs'],'m' if r['method']=='arun' else 'direction',
                         r['translation_run_mean_average'],r['translation_run_mean_sample_std'],
                         r['translation_run_mean_min'],r['translation_run_mean_max'],
                         r['failed_total'],r['timeout_total'],19445])
