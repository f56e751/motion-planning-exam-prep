# Motion Planning — Exam Prep

서울대 모션플래닝 강의 (H. Jin Kim) L00–L12 기말 시험 대비 자료.

시험 조건: cheat sheet 지참 가능, 계산기 사용 불가.

## 파일 구성

| 파일 | 설명 |
|---|---|
| `cheatsheet.md` / `.tex` / `.pdf` | 양면 A4 cheat sheet (24개 섹션, 강의안 매핑) |
| `exam_questions.md` / `.tex` / `.pdf` | 강의안별 출제 가능성 높은 28문제 + 풀이, 5단계 별표 티어 |

## Cheat Sheet
- 양면 A4 빽빽 layout. PDF가 시험장 지참용.
- 24개 섹션 (Path Planning부터 NMPC까지)
- 각 섹션에 강의안 번호 `[L##]` 매핑

## Exam Questions
- 강의안 L00–L12별로 메인 + 보조 2문제씩 (총 28).
- 풀이는 문제 바로 아래.
- 별표 5단계로 출제 가능성:
  - ★★★★★ 거의 확정적 출제 (2)
  - ★★★★ 매우 높음 (5)
  - ★★★ 높음 (6)
  - ★★ 중간 (7)
  - ★ 낮음 (8)

## 빌드 (PDF 재생성)

```
xelatex cheatsheet.tex
xelatex exam_questions.tex   # 두 번 실행 (목차 갱신)
```

XeLaTeX + kotex 필요 (한글 지원).
