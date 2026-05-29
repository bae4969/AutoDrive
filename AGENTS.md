# Codex 멀티에이전트 베이스 — lite/full 듀얼 모드 (Codex 진입점 · Claude 공존)

이 문서는 이 작업 디렉토리의 **Codex 하네스** 진입점이다 (같은 디렉토리에 Claude 하네스가 공존 — 아래 "공존 베이스 안내" 참조). 기본은 **lite 모드**이며, 사용자가 요청 앞에 `/full`을 붙였을 때만 **서칭 → 기획 → 구현 → 검증** 4단계 사이클로 진입한다.

이 하네스는 `AGENTS.md`와 `.codex/` 문서만 사용한다. 같은 디렉토리에 Claude 하네스(`CLAUDE.md`+`.claude/`)가 공존하지만, Codex는 그쪽의 세션 hook·subagent 디스커버리에 의존하지 않는다 (워크플로우 상태·이력만 `.workflow/state/`에서 공유 — 바로 아래 "공존 베이스 안내" 참조). Codex가 직접 역할을 전환해 수행하며, multi-agent 도구가 실제로 제공되는 환경에서만 같은 계약으로 보조 에이전트를 사용할 수 있다. 상태와 사용자 결정은 항상 메인 Codex가 관리한다.

## 공존 베이스 안내 (Codex·Claude parallel)

이 작업 디렉토리에는 **두 하네스가 나란히 공존**한다:

| 엔진 | 진입점 | 하네스 폴더 | engine 값 |
|---|---|---|---|
| Codex (본 하네스) | `AGENTS.md` | `.codex/` | `codex` |
| Claude | `CLAUDE.md` | `.claude/` | `claude` |

- **공유 SoT**: 워크플로우 상태(`current.json`)와 실행 이력(`history/`)은 `.workflow/state/`에서 **두 엔진이 공유**한다. 어느 엔진으로 진입하든 같은 상태·이력을 본다. 스키마 SoT는 [`.workflow/state/README.md`](.workflow/state/README.md).
- **본인 엔진 = `codex`**: state/history write 시 `engine` 필드와 `run_id`(`<ISO>__codex__<slug>`)에 항상 `codex`를 박는다.
- **한 시점 한 엔진(공존, 비동시)**: 같은 작업을 둘이 동시에 돌리는 병렬 실행이 아니다. `current.json`은 하나뿐이며, 세션 시작 시 `engine` 필드로 직전 워크플로우의 출처를 확인한다. 다른 엔진(`claude`)이 진행하던 in-progress 상태를 발견하면, 이어받기 전에 그 사실을 사용자에게 알리고 재개/재계획/취소를 묻는다.
- **규약은 엔진별 복제**: `.codex/`와 `.claude/`는 각자 4단계 워크플로우·코딩 원칙·역할 문서를 **독립 복제본**으로 가진다. 한쪽 규약을 고치면 다른 쪽도 맞춰야 일관된다 (공유하는 것은 `.workflow/state/`와 `temp/`뿐).
- **temp 공유**: `temp/`(plan·progress·report·input·output)도 두 엔진이 공유하는 작업공간이다.

## 모든 세션 시작 시 — 절대 진입 절차

1. **`.workflow/state/current.json`을 먼저 확인**한다.
   - `status`가 `searching` / `planning` / `awaiting_approval` / `approved` / `implementing` / `verifying`이면 신규 작업 전에 사용자에게 `재개 / 처음부터 재계획 / 취소`를 묻고 대기한다.
   - `status`가 `blocked`이면 차단 사유를 표시하고 `계획 수정 / 단계 건너뛰기 / 취소`를 묻고 대기한다.
   - `status`가 `done` / `aborted` / `{}` / 파일 없음이면 신규 요청으로 처리한다.
2. 신규 요청은 **모드 결정**부터 한다.
   - 요청이 `/full`로 시작하면 `mode=full`로 보고 [`.codex/rules/main_full_procedure.md`](.codex/rules/main_full_procedure.md)를 따른다.
   - `/full` prefix가 없으면 `mode=lite`로 처리한다.
   - 모드 결정은 `/full` prefix 매칭만 사용한다. 작업이 쉬워 보인다는 이유로 full을 lite로 낮추거나 lite를 full로 올리지 않는다.
3. 단순 조회, 상태 요약, 명백한 typo 같은 1줄 작업은 lite 안에서 즉시 처리한다.

## 모드별 사이클

### Lite 모드 (기본)

```text
사용자 요청
  ↓
[수정]  Codex가 직접 외과적으로 변경
  ↓
[검증]  critic 역할로 자체 검증 또는 독립 검증 도구 사용
   ├─ Pass + recommend_full:false → 짧게 보고 후 종료
   ├─ Pass + recommend_full:true  → 보고 + /full 재실행 권장
   └─ Fail → critic 사유로 자동 재수정 1회 → 재검증
        ├─ Pass → 종료
        └─ Fail → 사용자에게 "한 번 더 자동 수정 / /full로 승격 / 중단" 질문
```

- lite에서는 `.workflow/state/current.json`을 생성·갱신하지 않는다.
- 종료 시에는 가능하면 `.workflow/state/history/<run_id>/run.json`과 `index.jsonl`에 이력을 남긴다.
- `temp/plan.md`는 만들지 않는다. 긴 진행 기록이나 보고가 필요하면 `temp/progress.md`, `temp/report.md`, `temp/output/`을 사용한다.

### Full 모드 (`/full` prefix)

```text
/full 사용자 요청
  ↓
[1 SEARCH]    researcher 역할 — 코드/문서/웹 조사
  ↓ 사용자 확인
[2 PLAN]      planner 역할 — 구현 계획, 완료 조건, 금지 사항
  ↓ 사용자 승인 필수
[3 IMPLEMENT] implementer 역할 — Codex가 직접 파일 변경 및 자체 검증
  ↓
[4 VERIFY]    critic 역할 — 독립 검증
   ├─ Pass → REPORT → state.status=done
   └─ Fail → retry_count += 1
        ├─ retry_count < 3 → 사용자에게 재구현/방향 변경/중단 질문
        └─ retry_count == 3 → state.status=blocked, 사용자 결정 대기
```

매 단계 전이마다 `.workflow/state/current.json`을 갱신해 세션이 끊겨도 재개 가능하게 한다. 사용자 승인 없이는 Phase 3 구현에 들어가지 않는다.

## 역할 문서

| 역할 | 문서 | 사용 모드 | 책임 |
|---|---|---|---|
| researcher | [`.codex/roles/researcher.md`](.codex/roles/researcher.md) | full | 읽기 전용 조사 |
| planner | [`.codex/roles/planner.md`](.codex/roles/planner.md) | full | 계획, 완료 조건, 금지 사항 정의 |
| implementer | [`.codex/roles/implementer.md`](.codex/roles/implementer.md) | full | 파일 변경과 자체 검증 |
| critic | [`.codex/roles/critic.md`](.codex/roles/critic.md) | lite/full | 산출물 검증과 Pass/Fail 판정 |

Codex 구성에서는 로컬 역할 파일이 자동 subagent로 등록된다고 가정하지 않는다. 위 파일들은 **역할 프롬프트/체크리스트**다. full 모드에서는 메인 Codex가 순서대로 해당 역할을 수행한다. 실제 multi-agent 도구가 사용 가능하면 역할 문서의 입력/출력 계약을 그대로 전달하되, 상태 파일과 사용자 승인 관리는 메인 Codex만 한다.

## temp/ 폴더 규약

- `temp/input/`: 사용자가 제공한 참고 자료 위치. 사용자가 `input/foo.md`라고 말하면 `temp/input/foo.md`를 우선 확인한다.
- `temp/output/`: 사용자가 읽을 별도 산출물 위치. 긴 문서·보고서는 여기에 작성하고 채팅에는 링크와 짧은 요약만 남긴다.
- `temp/plan.md`: full Phase 2 계획서. 매 run 덮어쓴다.
- `temp/progress.md`: full 진행 로그. run 시작 시 초기화하고 단계마다 append한다.
- `temp/report.md`: 완료 보고 본문. 영속본은 `.workflow/state/history/<run_id>/run.json`의 `report_body`에 흡수한다.
- 사용자 요청 없이는 `.gitignore`를 수정하지 않는다.

## 코딩 행동 규칙

모든 변경에는 [`.codex/rules/coding_principles.md`](.codex/rules/coding_principles.md)의 4원칙을 적용한다.

1. 생각하고 코딩하기
2. 단순함 우선
3. 외과적 변경
4. 목표 주도 실행

구현 보고에는 변경 파일, 실행한 검증, 남은 위험을 짧게 적는다. full 모드 implementer는 `단순함`과 `외과적 변경`을 어떻게 지켰는지 한 줄씩 자체 점검한다.

## 사용자 질문 방식

사용자 결정이 필요한 모든 분기에서는 가능한 경우 Codex의 구조화된 사용자 입력 도구를 사용한다. 해당 도구가 없거나 현재 모드에서 사용할 수 없으면, 채팅에서 짧고 명확한 질문 하나로 멈춘다. 승인이 필요한 full Phase 2, 검증 실패 후 재시도, blocked 해소는 자동으로 넘기지 않는다.

## 명령 prefix 인덱스

이 하네스의 명령은 Codex 내장 slash command가 아니라 **사용자 입력 prefix 규약**이다.

- [`/full`](.codex/commands/full.md): full 4단계 사이클로 실행
- [`/draft`](.codex/commands/draft.md): 모호한 요청을 한 줄 실행 요청으로 정리
- [`/init`](.codex/commands/init.md): 새 프로젝트 진입 시 의도와 규칙 정리
- [`/export-team`](.codex/commands/export-team.md): 하네스 파일을 export 패키지로 복사
- [`/import-team`](.codex/commands/import-team.md): export 패키지를 현재 작업공간에 흡수

## 변경 시 원칙

- Codex 하네스의 자립성을 유지한다. `AGENTS.md`와 `.codex/`만으로 동작하며, 공존하는 `.claude/`의 hook·subagent 디스커버리 등 Claude 전용 설정에 의존하지 않는다 (공유는 `.workflow/state/`·`temp/`뿐).
- full 재시도 임계값은 3회다. 바꾸려면 `AGENTS.md`, `workflow_4stage.md`, `main_full_procedure.md`, `.workflow/state/README.md`를 함께 갱신한다.
- lite 자동 재수정은 1회다. 바꾸려면 `AGENTS.md`와 `workflow_4stage.md`를 함께 갱신한다.
- state/history 쓰기 책임자는 항상 메인 Codex다.