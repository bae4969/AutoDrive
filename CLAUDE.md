# 멀티에이전트 베이스 — lite/full 듀얼 모드 (Claude 진입점 · Codex 공존)

이 작업 디렉토리는 **기본 lite 모드(메인이 직접 Edit/Write → critic 검증)**로 동작하며, `/full` 슬래시 커맨드 사용 시에만 **4단계(서칭→기획→구현→검증)**로 진입하는 멀티에이전트 베이스다. lite에서는 메인이 외과적으로 직접 수정한 뒤 `critic`만 호출해 결과를 검증한다. `/full` 시에는 메인이 [`.claude/rules/main_full_procedure.md`](.claude/rules/main_full_procedure.md)를 따라 researcher/planner/coder/critic을 차례로 호출하며, 절차 본문은 [`.claude/agents/orchestrator.md`](.claude/agents/orchestrator.md)에 보존된다 (서브에이전트가 아닌 참조 절차서). 이 디렉토리는 Claude·Codex **공존 베이스**다 — Claude는 본 `CLAUDE.md`+`.claude/`로, Codex는 `AGENTS.md`+`.codex/`로 진입하며, 워크플로우 상태·이력은 `.workflow/state/`에서 **공유**한다 (바로 아래 "공존 베이스 안내" 참조).

## 공존 베이스 안내 (Claude·Codex parallel)

이 작업 디렉토리에는 **두 하네스가 나란히 공존**한다:

| 엔진 | 진입점 | 하네스 폴더 | engine 값 |
|---|---|---|---|
| Claude (본 하네스) | `CLAUDE.md` | `.claude/` | `claude` |
| Codex | `AGENTS.md` | `.codex/` | `codex` |

- **공유 SoT**: 워크플로우 상태(`current.json`)와 실행 이력(`history/`)은 `.workflow/state/`에서 **두 엔진이 공유**한다. 어느 엔진으로 진입하든 같은 상태·이력을 본다. 스키마 SoT는 [`.workflow/state/README.md`](.workflow/state/README.md).
- **본인 엔진 = `claude`**: state/history write 시 `engine` 필드와 `run_id`(`<ISO>__claude__<slug>`)에 항상 `claude`를 박는다.
- **한 시점 한 엔진(공존, 비동시)**: 같은 작업을 둘이 동시에 돌리는 병렬 실행이 아니다. `current.json`은 하나뿐이며, 세션 시작 시 `engine` 필드로 직전 워크플로우의 출처를 확인한다. 다른 엔진(`codex`)이 진행하던 in-progress 상태를 발견하면, 이어받기 전에 그 사실을 사용자에게 알리고 재개/재계획/취소를 묻는다.
- **규약은 엔진별 복제**: `.claude/`와 `.codex/`는 각자 4단계 워크플로우·코딩 원칙·역할 문서를 **독립 복제본**으로 가진다. 한쪽 규약을 고치면 다른 쪽도 맞춰야 일관된다 (공유하는 것은 `.workflow/state/`와 `temp/`뿐).
- **temp 공유**: `temp/`(plan·progress·report·input·output)도 두 엔진이 공유하는 작업공간이다.

## 모든 세션 시작 시 — 절대 진입 절차

1. **`.workflow/state/current.json` 먼저 확인** (워크플로우 재개 여부 + `engine` 출처 판단).
   - `status`가 `searching` / `planning` / `awaiting_approval` / `approved` / `implementing` / `verifying` → 사용자에게 재개·재계획·취소 옵션 질의(AskUserQuestion) 후 지시 대기.
   - `status`가 `blocked` → 차단 사유 표시 후 AskUserQuestion으로 "계획 수정 / 단계 건너뛰기 / 취소" 옵션 질의.
   - `status`가 `done` / `aborted` / 비어 있음 → 신규 요청으로 처리.
2. 신규 요청 처리 시 **모드 결정**:
   - 요청이 `/full`로 시작하면 → `mode=full`, 메인이 [`main_full_procedure`](.claude/rules/main_full_procedure.md)를 따라 4단계(researcher→planner→coder→critic) 실행.
   - `/full` prefix 없으면 → `mode=lite`, 메인이 직접 Edit/Write로 외과적 수정 후 `critic` 서브에이전트만 호출.
   - lite vs full 모드 결정은 `/full` slash prefix 매칭으로만 한다(다른 slash command — `/draft`, `/init` 등 — 은 자체 동작을 가지며 lite/full 모드 자체를 바꾸지 않는다). LLM이 "이 요청은 간단해 보인다" 같은 휴리스틱으로 모드를 바꾸는 것 금지.
3. lite 작업도 비단순 요청이면 본 사이클을 따른다. 단순 조회·typo는 메인이 즉시 처리(workflow_4stage.md "음성 예시" 참조).

## 모드별 사이클

### Lite 모드 (기본, `/full` prefix 없을 때)

```
유저 요청
  ↓
[수정]    메인이 직접 Edit/Write로 외과적 수정
  ↓
[검증]    critic 서브에이전트
   ├─ Pass + recommend_full:false → 메인이 한 줄 보고 → 종료
   ├─ Pass + recommend_full:true  → 메인이 보고 + 사용자에게 /full 재실행 안내
   └─ Fail → 메인이 critic 사유 받아 **자동 재수정 1회** → critic 재호출
        ├─ 재호출도 Fail → AskUser(자동 재구현 한번 더 / /full로 승격 / 중단)
        └─ 재호출 Pass → 종료
```

- 재시도 카운터·`blocked` 상태는 lite에서 쓰지 않는다 (full만 사용).
- `state.json`은 lite에서 생성·갱신하지 않는다. lite 종료 시에만 메인이 `history/<run_id>/run.json`을 작성한다 (스키마 동일).
- `temp/plan.md`는 lite에서 만들지 않는다. `temp/progress.md`·`temp/report.md`는 메인이 직접 작성.

### Full 모드 (`/full` prefix 사용 시)

```
# 절차서: main_full_procedure.md, 본문: orchestrator.md (비활성 참조)
/full 유저 요청
  ↓ (메인이 main_full_procedure 진입, mode=full)
[1 서칭]   메인 ─Agent→ researcher    → AskUser(범위 확정/추가 서칭)
  ↓
[2 기획]   메인 ─Agent→ planner       → AskUser(승인 / 수정 / 취소)  ← 승인 없이는 구현 진입 금지
  ↓
[3 구현]   메인 ─Agent→ coder         → Edit/Write로 파일 직접 변경, Bash로 자체 검증
  ↓
[4 검증]   메인 ─Agent→ critic
   ├─ Pass → REPORT → state.status=done, 종료
   └─ Fail → state.retry_count += 1
        ├─ retry_count < 3 : AskUser(자동 재구현 / 방향 변경 / 중단) → [3]로 복귀
        └─ retry_count == 3: 루프 중단, state.status=blocked
                              AskUser(계획 수정 / 단계 건너뛰기 / 취소)
```

매 단계 전이마다 `.workflow/state/current.json`을 갱신해 세션이 끊겨도 재개 가능.

## 구성 — 5개 서브에이전트 (full 전용 4개 + lite/full 공용 1개)

| 에이전트 | 도구 | 역할 | 사용 모드 | 권한 |
|---|---|---|---|---|
| [orchestrator](.claude/agents/orchestrator.md) | Read/Grep/Glob/Agent/TodoWrite/Edit/Write/AskUserQuestion | 4단계 순서·분배·상태 관리 | **참조 절차서**(비활성 — 메인이 호출하지 않음, 절차 본문 SoT 보존용) | 읽기 + Agent 호출 + state·메모리 **메타데이터만 Edit/Write** |
| [researcher](.claude/agents/researcher.md) | Read/Grep/Glob/WebSearch/WebFetch | 코드·웹·MCP 조사 | **full 전용** | 읽기 전용 + WebSearch/WebFetch/MCP |
| [planner](.claude/agents/planner.md) | Read/Grep/Glob | 설계·작업 분해·승인 카드 작성 | **full 전용** | 읽기 전용 |
| [coder](.claude/agents/coder.md) | Read/Bash/Glob/Grep/**Edit/Write/NotebookEdit** | 코드·파일 변경(처리), 자체 검증 | **full 전용** | full 모드에서 Edit/Write를 가진 유일한 서브에이전트 |
| [critic](.claude/agents/critic.md) | Read/Grep/Glob/Bash | 산출물 검증·테스트·요구사항 일치 판정 | **lite·full 공용** | 읽기 + Bash(테스트 실행) |

격리 메커니즘:
- **full 모드**: Edit/Write는 `coder` 서브에이전트에만 부여. researcher/planner/critic은 도구 자체가 없어 구조적으로 코드 편집 불가. 메인은 단계 전이용 메타데이터(`.workflow/state/**`, `temp/**` 등) 화이트리스트만 쓰고, 코드 파일은 손대지 않는다.
- **lite 모드**: 메인 에이전트가 자기 Edit/Write로 직접 외과적 수정. 자연어 규약상 lite 외 상황에서는 메인이 Edit/Write를 코드에 직접 쓰지 않는다(검증·재현 등으로 필요할 때 예외). critic이 사후에 "범위 외 변경" 사유로 잡아내는 게 안전망.

자세한 규약은 [.claude/rules/coding_principles.md](.claude/rules/coding_principles.md)와 각 에이전트 정의 파일 참조.

## temp/ 폴더 규약

`temp/` 디렉토리는 워크플로우 산출물과 사용자 자료의 임시 작업 공간이다.

- `temp/plan.md`는 Phase 2 기획서 본문이다. 매 run 덮어쓴다.
- `temp/progress.md`는 Phase 3/4 진행 상황이다. run 시작 시 초기화 후 append한다.
- `temp/report.md`는 Phase 5 완료 보고 본문이다. 매 run 덮어쓰며, 영속본은 `.workflow/state/history/<run_id>/run.json`의 `report_body` 필드에 흡수한다 (도구 가드가 `report.md` Write를 차단하므로 별도 파일을 만들지 않음).
- `temp/input/`은 사용자 자료실이다. 사용자가 `input/foo.md`처럼 언급하면 Claude는 `temp/input/foo.md`를 참조한다.
- `temp/output/`은 산출물 위치다. 사용자가 보고서/문서를 요청하면 Claude가 이곳에 작성한다.
- 채팅에는 마크다운 링크와 AskUserQuestion 카드만 띄운다. 본문은 항상 파일에 작성한다.
- `.gitignore`는 수정하지 않는다.

## 코딩 행동 규칙 (모든 단계 공통)

모든 코드 변경 — 특히 coder가 Edit/Write로 만드는 모든 변경 — 에는 [.claude/rules/coding_principles.md](.claude/rules/coding_principles.md)의 4원칙이 적용된다:

1. **생각하고 코딩하기** — 가정 명시, 해석 갈리면 옵션 제시, 모호하면 멈추고 질문.
2. **단순함 우선** — 요청되지 않은 기능·추상화·유연성 금지. 200줄→50줄 가능하면 다시 쓰기.
3. **외과적 변경** — 인접 코드 손대지 않기, 무관한 dead code는 언급만 (삭제 X), 변경된 모든 줄이 요청에 직접 트레이스되어야.
4. **목표 주도 실행** — 검증 가능한 성공 기준 정의, 다단계는 "단계 → 검증" 형태로 분해.

coder는 매 변경 보고 말미에 "행동 4원칙 자기 점검" 한두 줄을 적는다. critic은 위반을 결함(Defect) 사유로 명시한다.

## 호출 흐름 (DAG, 순환 금지)

### Lite 모드 (기본)

```
메인 ─Edit/Write→ (워크스페이스 변경)
  │
  └→ critic  (검증)
       ├─ Pass → 메인이 한 줄 보고 (+ recommend_full true면 /full 재실행 안내)
       └─ Fail → 메인이 critic 사유로 자동 재수정 1회 → critic 재호출
                  ├─ Pass → 종료
                  └─ Fail → AskUser(자동 재구현 한번 더 / /full로 승격 / 중단)
```

### Full 모드 (`/full` prefix)

```
# 절차서: main_full_procedure.md, 본문: orchestrator.md (비활성 참조)
메인 ─Agent→ researcher / planner / coder / critic  (차례로 호출, 단계 사이마다 AskUser·state 갱신)
              │           │         │       │
              │           │         │       └─ Pass/Fail verdict 반환
              │           │         └─ Edit/Write로 워크스페이스 변경 + Bash 자체 검증
              │           └─ 변경 명세 + 단계 분해 반환
              └─ 발견 사항 구조화 반환

검증 Fail 시: 메인 → AskUser → (재구현이면) coder → critic ...
```

- 서브에이전트끼리 직접 통신 금지. full에서는 모든 핸드오프가 메인을 거치고, lite에서는 메인이 critic만 직접 호출한다.
- full에서 메인은 main_full_procedure 절차에 따라 researcher/planner/coder/critic 4개를 차례로 호출한다. lite에서는 메인이 critic만 호출한다.

## AskUserQuestion 사용 원칙

사용자 응답이 필요한 **모든 분기**에서 AskUserQuestion으로 카드를 띄운다. 텍스트로 "Q1: ..., Q2: ..." 나열 금지. 한 호출에 최대 4개 질문. 카드를 띄우는 대표 시점:

- (full) 서칭 종료 후: 발견 사항 요약 + "기획 진입 / 추가 서칭 / 범위 변경"
- (full) 기획 종료 후: 계획 브리핑 + "승인 / 수정 / 취소" — **승인 없이는 구현 진입 금지**
- (full) 검증 1·2차 실패 후: 실패 사유 + "자동 재구현 / 방향 변경 / 중단"
- (full) 검증 3회째 실패 후: blocked 상태 + "계획 수정 / 단계 건너뛰기 / 취소"
- (lite) 자동 재수정 후에도 critic Fail: "자동 재구현 한번 더 / /full로 승격 / 중단"
- 모호한 요구사항 발견 시: 옵션 카드로 결정 받기

## 사용법

1. 작업할 디렉토리에서 세션 시작 (subagent 디스커버리는 cwd의 `.claude/agents/`만 본다).
2. 베이스를 다른 작업 디렉토리에 가져갈 때:
   - Claude 단독: `.claude/` 전체 + `CLAUDE.md` 복사 — Claude Code만 있으면 동작 (외부 CLI·인증 불필요)
   - 공존 전체(Claude+Codex): 위에 더해 `AGENTS.md` + `.codex/` 전체 + 공유 `.workflow/` 까지 복사
3. **기본 = lite 모드**: 요청을 그냥 입력하면 메인이 직접 외과적으로 수정한 뒤 critic 1회 검증.
4. **full 모드**: 요청 앞에 `/full`을 붙이면 researcher→planner→coder→critic 4단계로 처리. (자세한 내용: [.claude/commands/full.md](.claude/commands/full.md))
5. 단순 조회·typo 수정 같은 1줄 작업은 메인이 critic 호출 없이 직접 처리 (workflow_4stage.md "음성 예시" 참조).
6. **슬래시 커맨드 인덱스**:
   - [`/full`](.claude/commands/full.md) — lite → full 4단계 사이클 전환
   - [`/draft`](.claude/commands/draft.md) — 모호한 요청을 라운드 대화로 다듬어 한 줄 요청으로 조립
   - [`/init`](.claude/commands/init.md) — 새 프로젝트 진입 시 의도 파악 + memory/rules 기록
   - [`/export-team`](.claude/commands/export-team.md) — 베이스 초기상태를 타임스탬프 폴더로 export
   - [`/import-team`](.claude/commands/import-team.md) — export-team 패키지를 현 베이스에 흡수

## 변경 시 원칙

- 서브에이전트는 4개(researcher/planner/coder/critic)로 고정. orchestrator.md는 참조 절차서로 보존(서브에이전트로 호출하지 않음). 새 역할이 필요해도 같은 권한의 에이전트를 이름만 바꿔 늘리지 않는다.
- 읽기 전용과 쓰기 가능 권한을 항상 분리한다. **full 모드에서 Edit/Write는 `coder`에만 부여**한다 (메인의 full 모드 메타데이터 화이트리스트 예외 + lite 모드의 메인 직접 코드 편집 예외). 다른 서브에이전트(researcher/planner/critic)에 Edit/Write를 추가하지 않는다 — IMPLEMENT 격리의 핵심.
- 재시도 카운터 임계값은 **full만 사용**(3회). lite는 자동 재수정 1회 + 사용자 카드 1회로 고정. full 임계값을 바꾸려면 `workflow_4stage.md`, 본 파일, `.claude/agents/orchestrator.md`, `.claude/rules/main_full_procedure.md` 네 곳을 함께 갱신.
- 모드 분기 로직(슬래시 prefix 매칭)은 main_full_procedure.md와 CLAUDE.md 두 곳에 명시. 둘 다 함께 갱신.
- lite·full 모두 history write 책임자는 **메인** (full에서는 main_full_procedure 절차에 따라, 동일 스키마). 변경 시 `state/README.md` "쓰기 책임" 절도 함께 갱신.
