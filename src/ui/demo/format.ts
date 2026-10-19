import { reportMarkdown, reportRows } from '../../mission/autonomy/report';
import type { AutonomyReport, Phase, RunOutcome } from '../../mission/autonomy/contracts';

export interface PhaseStep {
  key: string;
  label: string;
  phases: Phase[];
}

/** The stepper groups runner phases into the story a viewer follows. */
export const PHASE_STEPS: PhaseStep[] = [
  { key: 'coverage', label: 'Coverage', phases: ['preflight', 'coverage'] },
  { key: 'approach', label: 'Approach', phases: ['approach'] },
  { key: 'inspect', label: 'Inspect', phases: ['inspect'] },
  { key: 'entry', label: 'Entry', phases: ['entry_select', 'await_operator'] },
  { key: 'fly_through', label: 'Fly-through', phases: ['fly_through'] },
  { key: 'indoor', label: 'Indoor', phases: ['indoor_explore'] },
  { key: 'confirm', label: 'Confirm', phases: ['confirm'] },
  { key: 'home', label: 'Return home', phases: ['exit', 'return_home', 'land', 'recover'] },
  { key: 'report', label: 'Report', phases: ['done'] },
];

export function stepIndex(phase: Phase): number {
  return PHASE_STEPS.findIndex((step) => step.phases.includes(phase));
}

export const PHASE_LABELS: Record<Phase, string> = {
  preflight: 'Preflight',
  coverage: 'Outdoor coverage',
  approach: 'Approach',
  inspect: 'Inspect openings',
  entry_select: 'Select entry',
  await_operator: 'Waiting for operator',
  fly_through: 'Fly-through',
  indoor_explore: 'Indoor search',
  confirm: 'Confirm contact',
  exit: 'Exit building',
  return_home: 'Return home',
  land: 'Landing',
  recover: 'Recover',
  done: 'Done',
};

export const OUTCOME_LABELS: Record<RunOutcome, string> = {
  found: 'Target found',
  clear: 'Area confirmed clear',
  not_found: 'Not found',
  incomplete: 'Incomplete',
  aborted: 'Aborted',
  landed: 'Landed early',
};

export function formatClock(sec: number): string {
  const s = Math.max(0, Math.floor(sec));
  return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`;
}

export function formatPct(value: number, digits = 0): string {
  return `${(value * 100).toFixed(digits)}%`;
}

export interface ReportRow {
  label: string;
  value: string;
}

export function rowsFor(report: AutonomyReport): ReportRow[] {
  return reportRows(report);
}

export function markdownFor(report: AutonomyReport): string {
  return reportMarkdown(report);
}

/** Display estimate of how safe an opening looks, for the map legend only. */
export function openingSafetyHint(clearWidth: number, clearHeight: number, passable: boolean): number {
  if (!passable) return 0;
  const margin = Math.min(clearWidth, clearHeight) - 0.38;
  return Math.max(0, Math.min(1, margin / 0.9));
}

export function safetyColor(safety: number): string {
  if (safety >= 0.6) return '#2fd27a';
  if (safety >= 0.3) return '#f2b134';
  return '#ef5350';
}

export const CLASS_COLORS: Record<string, string> = {
  person: '#49f0a0',
  backpack: '#8ec5ff',
  phone: '#d7c4ff',
  toolbox: '#ffb15a',
  medkit: '#ff7a8a',
  bottle: '#7ee0ff',
  radio: '#c6f08a',
  helmet: '#f0e2a0',
  blanket: '#e7c2ff',
  mannequin: '#d5dbe3',
  coat: '#e6c39a',
  decoy_pack: '#9bb7c9',
  decoy_case: '#c4a27a',
};

export function classLabel(classId: string): string {
  return classId.replaceAll('_', ' ');
}
