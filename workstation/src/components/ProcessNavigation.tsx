import type { ReactNode } from "react";

export type ProcessStep = 1 | 2 | 3 | 4;

// Navigation is separate from stage editors and execution. These are the current
// four views, not workflow instances: repeating/executing blocks is future work.
export const processStages: readonly { id: ProcessStep; title: string; icon: ReactNode }[] = [
  { id: 1, title: "스캔", icon: <><path d="M4 9V4h5m6 0h5v5M4 15v5h5m6 0h5v-5M7 12h10" /></> },
  { id: 2, title: "도장 경로 생성", icon: <><circle cx="5" cy="5" r="2" /><circle cx="19" cy="19" r="2" /><path d="M8 5h9a4 4 0 0 1 0 8H7a3 3 0 0 0 0 6h9" /></> },
  { id: 3, title: "도장 경로 수정", icon: <><path d="M3 6h4m4 0h10M3 12h10m4 0h4M3 18h4m4 0h10" /><circle cx="9" cy="6" r="2" /><circle cx="15" cy="12" r="2" /><circle cx="9" cy="18" r="2" /></> },
  { id: 4, title: "모션 플래닝", icon: <><path d="M5 21h12M7 21v-4h8v4M9 17l-4-7 7-6 5 5-5 5 1 3m4-8 3-2 2 3-3 2" /><circle cx="5" cy="10" r="2" /><circle cx="12" cy="4" r="2" /><circle cx="12" cy="14" r="2" /></> },
];

interface Props {
  current: ProcessStep;
  standalone: boolean;
  busy: boolean;
  completed: readonly ProcessStep[];
  canOpen: (step: ProcessStep) => boolean;
  onSelect: (step: ProcessStep) => void;
}

export function ProcessNavigation({ current, standalone, busy, completed, canOpen, onSelect }: Props) {
  return <nav className="process-navigation" aria-label="도장 공정 단계">
    <div className="navigation-heading">WORKFLOW</div>
    <ol className="process-stepper">
      {processStages.filter((item) => !standalone || item.id === 3).map((item) => {
        const done = completed.includes(item.id);
        return <li key={item.id}>
          <button type="button" className={`${current === item.id ? "active" : ""} ${done ? "completed" : ""}`}
            aria-current={current === item.id ? "step" : undefined}
            disabled={busy || !canOpen(item.id)} onClick={() => onSelect(item.id)}>
            <span className="step-number">{String(item.id).padStart(2, "0")}</span>
            {done && <span className="step-complete" aria-label="완료">✓</span>}
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.35" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">{item.icon}</svg>
            <strong>{item.title}</strong>
          </button>
        </li>;
      })}
    </ol>
    <div className="navigation-caption">{standalone ? "읽기 전용" : "공정 선택"}</div>
  </nav>;
}
