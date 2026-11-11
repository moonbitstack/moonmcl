// The forty-eight Reeds-Shepp words. A word is one of twelve families of
// segments applied to a pose in one of four frames: as given, reflected across
// the axis of travel, driven backwards, or both. Naming each word by its
// segments is what makes a path readable in a report and comparable with the
// published table.

///|
/// One of the forty-eight reversable-car path words.
#derive(Debug)
pub(all) enum ReedsSheppWord {
  ///
  /// Left, straight, left.
  LpSpLp
  ///
  /// Right, straight, right.
  RpSpRp
  ///
  /// Left, straight, left, driven backwards.
  LmSmLm
  ///
  /// Right, straight, right, driven backwards.
  RmSmRm
  ///
  /// Left, straight, right.
  LpSpRp
  ///
  /// Right, straight, left.
  RpSpLp
  ///
  /// Left, straight, right, driven backwards.
  LmSmRm
  ///
  /// Right, straight, left, driven backwards.
  RmSmLm
  ///
  /// Left, right, left.
  LpRmLp
  ///
  /// Right, left, right.
  RpLmRp
  ///
  /// Left, right, left, driven backwards.
  LmRpLm
  ///
  /// Right, left, right, driven backwards.
  RmLpRm
  ///
  /// Left, right, left with the last arc reversed.
  LpRmLm
  ///
  /// Right, left, right with the last arc reversed.
  RpLmRm
  ///
  /// Left, right, left, both arcs reversed.
  LmRpLp
  ///
  /// Right, left, right, both arcs reversed.
  RmLpRp
  ///
  /// Left, right, left with the middle arc reversed.
  LpRpLm
  ///
  /// Right, left, right with the middle arc reversed.
  RpLpRm
  ///
  /// Left, right, left, first two arcs reversed.
  LmRmLp
  ///
  /// Right, left, right, first two arcs reversed.
  RmRmLp
  ///
  /// Left, right, left, right: two arcs forward then two reversed.
  LpRpLmRm
  ///
  /// Right, left, right, left: two arcs forward then two reversed.
  RpLpRmLm
  ///
  /// Left, right, left, right: two arcs reversed then two forward.
  LmRmLpRp
  ///
  /// Right, left, right, left: two arcs reversed then two forward.
  RmRmLpLp
  ///
  /// Left, right, left, right: alternating from the start.
  LpRmLmRp
  ///
  /// Right, left, right, left: alternating from the start.
  RpLmRmLp
  ///
  /// Left, right, left, right: reversed, then alternating.
  LmRpLpRm
  ///
  /// Right, left, right, left: reversed, then alternating.
  RmLpRpLm
  ///
  /// Left, right, straight, left: a half turn between the arcs.
  LpRmSmLm
  ///
  /// Right, left, straight, right: a half turn between the arcs.
  RpLmSmRm
  ///
  /// Left, right, straight, left: reversed.
  LmRpSpLp
  ///
  /// Right, left, straight, right: reversed.
  RmLpSpRp
  ///
  /// Left, right, straight, right: a half turn between the arcs.
  LpRmSmRm
  ///
  /// Right, left, straight, left: a half turn between the arcs.
  RpLmSmLm
  ///
  /// Left, right, straight, right: reversed.
  LmRpSpRp
  ///
  /// Right, left, straight, left: reversed.
  RmLpSpLp
  ///
  /// Left, straight, right, left: a half turn before the last arc.
  LpSpRmLm
  ///
  /// Right, straight, left, right: a half turn before the last arc.
  RpSpLmRm
  ///
  /// Left, straight, right, left: reversed.
  LmSmRpLp
  ///
  /// Right, straight, left, right: reversed.
  RmSmLpRp
  ///
  /// Left, straight, left, right: a half turn before the last arc.
  LpSpLpRm
  ///
  /// Right, straight, right, left: a half turn before the last arc.
  RpSpRpLm
  ///
  /// Left, straight, left, right: reversed.
  LmSmLmRp
  ///
  /// Right, straight, right, left: reversed.
  RmSmRmLp
  ///
  /// Left, right, straight, left, right: half turns on both sides.
  LpRmSmLmRp
  ///
  /// Right, left, straight, right, left: half turns on both sides.
  RpLmSmRmLp
  ///
  /// Left, right, straight, left, right: reversed.
  LmRpSpLpRm
  ///
  /// Right, left, straight, right, left: reversed.
  RmLpSpRpLm
}

///|
/// The position of a word in the enumeration: family times four plus variant.
///
/// A family is one set of segment formulas and a variant is one of the four
/// frames it is applied in, so the pair is all the table a word needs.
pub fn ReedsSheppWord::code(self : ReedsSheppWord) -> Int32 {
  match self {
    LpSpLp => 0
    RpSpRp => 1
    LmSmLm => 2
    RmSmRm => 3
    LpSpRp => 4
    RpSpLp => 5
    LmSmRm => 6
    RmSmLm => 7
    LpRmLp => 8
    RpLmRp => 9
    LmRpLm => 10
    RmLpRm => 11
    LpRmLm => 12
    RpLmRm => 13
    LmRpLp => 14
    RmLpRp => 15
    LpRpLm => 16
    RpLpRm => 17
    LmRmLp => 18
    RmRmLp => 19
    LpRpLmRm => 20
    RpLpRmLm => 21
    LmRmLpRp => 22
    RmRmLpLp => 23
    LpRmLmRp => 24
    RpLmRmLp => 25
    LmRpLpRm => 26
    RmLpRpLm => 27
    LpRmSmLm => 28
    RpLmSmRm => 29
    LmRpSpLp => 30
    RmLpSpRp => 31
    LpRmSmRm => 32
    RpLmSmLm => 33
    LmRpSpRp => 34
    RmLpSpLp => 35
    LpSpRmLm => 36
    RpSpLmRm => 37
    LmSmRpLp => 38
    RmSmLpRp => 39
    LpSpLpRm => 40
    RpSpRpLm => 41
    LmSmLmRp => 42
    RmSmRmLp => 43
    LpRmSmLmRp => 44
    RpLmSmRmLp => 45
    LmRpSpLpRm => 46
    RmLpSpRpLm => 47
  }
}

///|
/// The word a code names, or `None` outside the forty-eight.
pub fn ReedsSheppWord::of_code(code : Int32) -> ReedsSheppWord? {
  match code {
    0 => Some(LpSpLp)
    1 => Some(RpSpRp)
    2 => Some(LmSmLm)
    3 => Some(RmSmRm)
    4 => Some(LpSpRp)
    5 => Some(RpSpLp)
    6 => Some(LmSmRm)
    7 => Some(RmSmLm)
    8 => Some(LpRmLp)
    9 => Some(RpLmRp)
    10 => Some(LmRpLm)
    11 => Some(RmLpRm)
    12 => Some(LpRmLm)
    13 => Some(RpLmRm)
    14 => Some(LmRpLp)
    15 => Some(RmLpRp)
    16 => Some(LpRpLm)
    17 => Some(RpLpRm)
    18 => Some(LmRmLp)
    19 => Some(RmRmLp)
    20 => Some(LpRpLmRm)
    21 => Some(RpLpRmLm)
    22 => Some(LmRmLpRp)
    23 => Some(RmRmLpLp)
    24 => Some(LpRmLmRp)
    25 => Some(RpLmRmLp)
    26 => Some(LmRpLpRm)
    27 => Some(RmLpRpLm)
    28 => Some(LpRmSmLm)
    29 => Some(RpLmSmRm)
    30 => Some(LmRpSpLp)
    31 => Some(RmLpSpRp)
    32 => Some(LpRmSmRm)
    33 => Some(RpLmSmLm)
    34 => Some(LmRpSpRp)
    35 => Some(RmLpSpLp)
    36 => Some(LpSpRmLm)
    37 => Some(RpSpLmRm)
    38 => Some(LmSmRpLp)
    39 => Some(RmSmLpRp)
    40 => Some(LpSpLpRm)
    41 => Some(RpSpRpLm)
    42 => Some(LmSmLmRp)
    43 => Some(RmSmRmLp)
    44 => Some(LpRmSmLmRp)
    45 => Some(RpLmSmRmLp)
    46 => Some(LmRpSpLpRm)
    47 => Some(RmLpSpRpLm)
    _ => None
  }
}

///|
/// The words of the family `family`, in variant order.
pub fn family_words(family : Int32) -> Array[ReedsSheppWord] {
  let words : Array[ReedsSheppWord] = []
  for variant in 0..<4 {
    match ReedsSheppWord::of_code(family * 4 + variant) {
      Some(word) => words.push(word)
      None => ()
    }
  }
  words
}
