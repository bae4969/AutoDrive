# .workflow/state/ — 4단계 워크플로우 상태 (공유 SoT, Claude·Codex 공존)

`current.json`은 **full 모드 활성 워크플로우의 진실의 원천(Source of Truth)**이다. 이 디렉토리는 **Claude(`.claude/` + `CLAUDE.md`)와 Codex(`.codex/` + `AGENTS.md`) 두 하네스가 공유**한다 — 어느 엔진으로 진입하든 같은 `current.json`·`history/`를 본다. 각 엔진의 메인만 (full 모드에서 각자의 `main_full_procedure`에 따라) 쓰기 권한이 있다. 서브에이전트(Claude)·역할(Codex)은 읽기만.

> **공존 전제**: 이 베이스는 한 시점에 한 엔진이 워크플로우를 진행하는 **공존(coexistence)** 모델이다 (같은 작업을 둘이 동시에 돌리는 병렬 실행이 아님). `current.json`은 하나뿐이고, `engine` 필드로 "지금 이 워크플로우를 누가 시작했는지"를 기록한다. 다른 엔진이 세션을 시작하면 `engine` 필드로 출처를 인지하고, in-progress 상태면 재개/재계획/취소를 사용자에게 먼저 묻는다.

**lite 모드는 current.json을 생성·갱신하지 않는다.** lite는 메인이 직접 외과적 수정 + critic 1회 검증으로 끝나는 단일 실행이라 재개·상태 추적이 필요 없기 때문이다. lite 종료 시점에는 메인이 `history/<run_id>/run.json`만 작성한다 (스키마 동일, 아래 history 절 참조).

## engine 필드 (공존 베이스 핵심)

- `current.json`·`run.json`·`index.jsonl` 모두 `"engine": "claude" | "codex"` 필드를 가진다.
- `run_id`에도 엔진 표식을 넣는다: `<ISO-timestamp>__<engine>__<slug>`. 같은 시각에 두 엔진이 각각 run을 남겨도 디렉토리·식별자가 충돌하지 않게 한다.
- 본인 엔진 값: **Claude 진입(`CLAUDE.md`) = `"claude"`, Codex 진입(`AGENTS.md`) = `"codex"`**. 메인은 state/history write 시 항상 자기 엔진을 박는다.

## 갱신 규칙

- **매 단계 전이마다 갱신** (배치로 미루지 않음). 세션이 어디서 끊겨도 손실을 한 단계 이내로 제한.
- 갱신 순서: **state.json 먼저 → TodoWrite 동기화**. 역순 금지. (TodoWrite가 없는 엔진은 state.json만.)
- 불일치 시 state.json을 신뢰, 보조 view를 재구성.

## 스키마 (current.json)

```json
{
  "request_id": "20260526_103045",
  "request": "사용자 요청 한 줄 요약",
  "engine": "claude",
  "mode": "full",
  "status": "searching|planning|awaiting_approval|approved|implementing|verifying|done|aborted|blocked",
  "current_phase": "search|plan|implement|verify",
  "retry_count": 0,
  "block_reason": null,
  "started_at": "2026-05-26T10:30:45+09:00",
  "completed_at": null,
  "decisions": [
    {
      "at": "...",
      "context": "검증 1회차 실패 후 사용자 확인",
      "question": "어떻게 진행할까요?",
      "options": ["자동 재구현", "방향 변경", "중단"],
      "chosen": "자동 재구현"
    }
  ]
}
```

## 상태 값 의미

| status | 의미 | 다음 가능한 전이 |
|---|---|---|
| `searching` | Phase 1 진행 중 | `planning`, `aborted` |
| `planning` | Phase 2 진행 중 | `awaiting_approval`, `aborted` |
| `awaiting_approval` | 기획 카드 띄우고 사용자 응답 대기 | `approved` (→ `implementing`), `planning` (수정요청), `aborted` |
| `approved` | 기획 승인 직후 (Phase 3 진입 전) | `implementing` |
| `implementing` | Phase 3 진행 중 | `verifying`, `aborted` |
| `verifying` | Phase 4 진행 중 | `done`, `implementing` (재구현), `planning` (방향 변경), `blocked`, `aborted` |
| `done` | 완료 | (종료) |
| `aborted` | 사용자 취소 | (종료) |
| `blocked` | 재시도 3회 실패 — 사용자 결정 대기 | `planning`, `done` (건너뛰기), `aborted` |

## 세션 재개

세션 시작 시 각 엔진의 메인은 다음 절차:

1. `current.json` Read.
2. `engine` 필드로 출처 확인. 본인과 다른 엔진이 시작한 워크플로우면, 이어받기 전에 사용자에게 그 사실을 알린다 (예: "이전 워크플로우는 codex가 진행 중이었습니다").
3. `status` 분기:
   - 비어 있음 / `done` / `aborted` → 신규 요청 처리.
   - in-progress 상태(`searching` ~ `verifying`) → 재개 / 처음부터 재계획 / 취소 질의.
   - `blocked` → 차단 사유 표시 + 계획 수정 / 단계 건너뛰기 / 취소 질의.
4. 재개 시 이미 처리된 단계는 다시 실행하지 않는다. `current_phase` 값을 신뢰하되, 의심스러우면 `temp/progress.md`로 보조 확인.

## history/ 영속화

모든 워크플로우 종료(done/aborted/blocked)는 `.workflow/state/history/`에 기록된다. 두 엔진이 같은 history를 공유하므로, `index.jsonl`을 보면 claude·codex 실행이 한 타임라인에 섞여 누적된다 (`engine` 필드로 구분).

### 디렉토리 구조

```
.workflow/state/history/
├── index.jsonl                    # 전체 실행 목록 (한 줄 = 한 run, append-only, claude·codex 공통)
└── <run_id>/
    └── run.json                   # 실행 메타데이터 + report_body (보고서 본문 흡수)
```

> 보고서 본문은 별도 `report.md` 파일이 아니라 `run.json`의 `report_body` 필드에 흡수한다. 하니스 도구 가드가 `report.md` Write를 차단하므로, run당 단일 파일(run.json)로 영속화해 가드와 무관하게 일관성을 보장한다.

`run_id` 형식: `<ISO-timestamp>__<engine>__<slug>` (예: `2026-05-26T10-30-00__claude__add-history-store`). slug는 요청 첫 줄 kebab-case 30자 컷. `<engine>`은 `claude` 또는 `codex`.

### run.json 스키마

```json
{
  "run_id": "2026-05-26T10-30-00__claude__add-history-store",
  "engine": "claude",
  "started_at": "2026-05-26T10:30:00+09:00",
  "completed_at": "2026-05-26T10:45:23+09:00",
  "status": "done|aborted|blocked",
  "mode": "full|lite",
  "request": "사용자 원문 요청 (또는 한 줄 요약)",
  "plan_summary": "확정한 접근 한 단락",
  "retry_count": 0,
  "final_verdict": "Pass|Fail|N/A",
  "report_body": "Phase 5 보고서 본문 (temp/report.md 그대로). aborted/blocked 시 사유 단락."
}
```

- `engine`: `claude` | `codex` — 본 run을 실행한 엔진. run_id 표식과 일치해야 한다.
- `status`: `done` | `aborted` | `blocked`
- `mode`: `full` | `lite` — 본 run을 생성한 진입 경로.
- `final_verdict`: `Pass` | `Fail` | `N/A` (aborted/blocked 시 N/A; status=done일 때는 N/A 금지 — Pass 또는 Fail만)
- `report_body`: Phase 5 완료 보고 본문(`temp/report.md`와 동일). aborted/blocked 시 종료 사유 단락. 별도 `report.md` 파일을 만들지 않는다 (도구 가드 회피).

### index.jsonl 한 줄 스키마

```json
{"run_id":"2026-05-26T10-30-00__claude__add-history-store","engine":"claude","status":"done","mode":"full","request":"변경 이력 저장 구조 추가","files_changed":2,"retry_count":0,"final_verdict":"Pass"}
```

### 쓰기 규칙

- **full**: 각 엔진의 메인이 history에 쓴다 (Claude는 `main_full_procedure` §6, Codex는 `.codex/rules/main_full_procedure.md`). 서브에이전트(researcher/planner/coder/critic)·역할(researcher/planner/implementer/critic)은 직접 쓰지 않는다. 쓰는 시점은 state.status가 done/aborted/blocked로 확정되는 전이에서.
- **lite**: 메인이 직접 쓴다. critic 검증 종료(Pass 또는 Fail 보고 종료) 시점에 `<run_id>/run.json` + `index.jsonl` 한 줄 append. lite의 `plan_summary`는 "lite — 메인 직접 수정" 같은 한 줄로 채워 식별. retry_count는 자동 1회 + 사용자 카드 1회의 시도 횟수(최대 2)를 적는다.
- **engine 일관성**: 어느 엔진이든 자기 `engine` 값을 `run.json`·`index.jsonl`·`run_id`에 동일하게 박는다.
- **불변성**: 이미 작성된 `<run_id>/run.json`은 사후 정정·편집하지 않는다. `index.jsonl`은 append-only — 기존 라인 수정·삭제 금지.
- **보존**: 무제한, 자동 청소 없음. 삭제는 사용자 명시 지시가 있을 때만.
