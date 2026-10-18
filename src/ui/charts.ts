import uPlot from 'uplot';
import 'uplot/dist/uPlot.min.css';

export class RewardChart {
  private plot: uPlot;
  private xs: number[] = [];
  private ys: number[] = [];

  constructor(el: HTMLElement, color = '#3dff9a') {
    this.plot = new uPlot(
      {
        width: Math.max(160, el.clientWidth || 420),
        height: 150,
        cursor: { show: false },
        legend: { show: false },
        scales: { x: { time: false } },
        axes: [
          { stroke: '#7f90a8', grid: { stroke: 'rgba(127,144,168,0.15)' }, ticks: { stroke: '#7f90a8' } },
          { stroke: '#7f90a8', grid: { stroke: 'rgba(127,144,168,0.12)' }, ticks: { stroke: '#7f90a8' } },
        ],
        series: [{}, { stroke: color, width: 2, fill: 'rgba(61,255,154,0.08)' }],
      },
      [[], []],
      el,
    );
  }

  push(x: number, y: number): void {
    this.xs.push(x);
    this.ys.push(y);
    if (this.xs.length > 400) {
      this.xs.shift();
      this.ys.shift();
    }
    this.plot.setData([this.xs, this.ys]);
  }

  resize(width: number): void {
    this.plot.setSize({ width, height: 150 });
  }
}
