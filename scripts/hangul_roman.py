# hangul_roman.py — 한글 지역 이름 → 폴더용 영문 이름 (인터넷 없이).
#
#   region_to_english("강남역 4번출구") → "gangnam_station_4th_entrance"
#   region_to_english("까치산")         → "kkachisan"
#   region_to_english("gangnam")        → "gangnam"
#
# 글자 변환은 국어의 로마자 표기법(연음 · 비음화 · 유음화까지). 소리가 덧나는 이름(학여울 등)처럼
# 규칙과 공식 표기가 다른 곳은 PLACE_NAMES 에 적어 두면 그쪽이 먼저 쓰인다.

import re

# 공식 표기가 규칙과 다른 지역 — 필요할 때 여기에 추가
PLACE_NAMES = {
    "학여울": "hangnyeoul",
}

INITIALS = ["g", "kk", "n", "d", "tt", "r", "m", "b", "pp", "s", "ss", "", "j", "jj", "ch", "k", "t", "p", "h"]
MEDIALS = ["a", "ae", "ya", "yae", "eo", "e", "yeo", "ye", "o", "wa", "wae", "oe", "yo", "u", "wo", "we",
           "wi", "yu", "eu", "ui", "i"]
# 받침: (끝소리 무리, 뒤에 모음이 오면 넘어가는 소리)
FINALS = [(None, ""), ("k", "g"), ("k", "kk"), ("k", "gs"), ("n", "n"), ("n", "nj"), ("n", "nh"),
          ("t", "d"), ("l", "r"), ("k", "lg"), ("m", "lm"), ("l", "lb"), ("l", "ls"), ("l", "lt"),
          ("p", "lp"), ("l", "lh"), ("m", "m"), ("p", "b"), ("p", "bs"), ("t", "s"), ("t", "ss"),
          ("ng", "ng"), ("t", "j"), ("t", "ch"), ("k", "k"), ("t", "t"), ("p", "p"), ("t", "")]
NASAL = {"k": "ng", "t": "n", "p": "m"}
I_N, I_R, I_M, I_NONE = 2, 5, 6, 11      # ㄴ ㄹ ㅁ ㅇ


def _split(ch):
    code = ord(ch) - 0xAC00
    return code // 588, code % 588 // 28, code % 28


def romanize(word):
    """한글 한 낱말 → 로마자. 한글·영문·숫자 밖의 글자는 버린다."""
    out = []
    syl = [_split(c) if "가" <= c <= "힣" else c for c in word]
    for i, s in enumerate(syl):
        if isinstance(s, str):
            if s.isascii() and s.isalnum():
                out.append(s.lower())
            continue
        ini, med, fin = s
        prev = syl[i - 1] if i and not isinstance(syl[i - 1], str) else None
        nxt = syl[i + 1] if i + 1 < len(syl) and not isinstance(syl[i + 1], str) else None
        head = INITIALS[ini]
        if prev and ini == I_R and FINALS[prev[2]][0] is not None:
            head = "l" if FINALS[prev[2]][0] in ("n", "l") else "n"     # 신림 sillim, 정릉 jeongneung
        if prev and ini == I_N and FINALS[prev[2]][0] == "l":
            head = "l"                                                  # 설날 seollal
        if prev and ini == I_NONE and FINALS[prev[2]][0] is not None:
            head = ""                                                   # 받침이 넘어왔다
        group, carry = FINALS[fin]
        if group is None:
            tail = ""
        elif nxt and nxt[0] == I_NONE:
            tail = carry
        elif nxt and nxt[0] == I_R:
            tail = "l" if group in ("n", "l") else NASAL.get(group, group)
        elif nxt and nxt[0] in (I_N, I_M):
            tail = "l" if group == "l" and nxt[0] == I_N else NASAL.get(group, group)
        else:
            tail = group
        out.append(head + MEDIALS[med] + tail)
    return "".join(out)


def ordinal(n):
    n = int(n)
    suffix = "th" if 10 <= n % 100 <= 20 else {1: "st", 2: "nd", 3: "rd"}.get(n % 10, "th")
    return f"{n}{suffix}"


def _word(w):
    if w in PLACE_NAMES:
        return PLACE_NAMES[w]
    m = re.fullmatch(r"(\d+)번(?:출구)?", w)
    if m:
        return ordinal(m.group(1)) + "_entrance"
    if len(w) > 1 and w.endswith("역"):
        stem = w[:-1]
        return f"{PLACE_NAMES.get(stem) or romanize(stem)}_station"
    return romanize(w)


def region_to_english(text):
    """지역 이름 → 폴더에 쓸 영문 (소문자 · 숫자 · _ 만). 이미 영문이면 소문자로만."""
    text = re.sub(r"(\d+)\s*번\s*출구", r" \1번출구 ", text.strip())     # 4 번 출구 → 4번출구 (한 낱말)
    text = re.sub(r"(\S)역(?=\d)", r"\1역 ", text)                         # 강남역4번출구 → 강남역 4번출구
    words = [w for w in re.split(r"[\s_\-/,.]+", text) if w]
    out = "_".join(filter(None, (_word(w) for w in words)))
    return re.sub(r"_+", "_", out).strip("_")


if __name__ == "__main__":
    import sys
    for w in sys.argv[1:] or ["강남역 4번출구", "강남역4번 출구", "까치산", "선정릉역", "성수", "왕십리역 11번출구",
                              "학여울역 2번출구", "Gangnam", "신림 1번출구", "독립문"]:
        print(w, "→", region_to_english(w))
