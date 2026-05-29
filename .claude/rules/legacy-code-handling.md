# legacy-code-handling — 비활성/롤백 잔재 코드

이 프로젝트에는 비활성화·롤백된 잔재 코드가 있다(예: `PiCar::initRobotHat()`의 주석 처리된 CameraMotor 초기화, gpiod→wiringPi 롤백 잔재, `Basic.h`의 중복 핀 번호). 이를 **정리 대상으로 보되 명시 지시 없이 삭제하지 않는다**.

**Why:** 잔재 코드가 의도적 비활성인지 미완성/롤백 흔적인지 코드만으로는 단정하기 어렵다. 테스트·CI가 없어 삭제가 만든 회귀를 자동으로 못 잡으므로, 무단 정리는 위험 대비 이득이 작다.

**How to apply:**
- 작업 중 dead code/비활성 코드를 발견하면 보고서·변경 설명에 **언급만** 한다. 지시 없이 삭제하지 않는다([coding_principles.md](coding_principles.md) §3 외과적 변경과 동일 기조).
- 사용자가 명시적으로 "정리/삭제"를 지시하면 그때 제거한다.
- 본인 변경으로 새로 생긴 orphan(미사용 import/변수)만은 정리해도 된다 — 사전 존재 dead code와 구분.

관련 사실: [[project-intent]].
