// The wire format between the documentation page and this build.
//
// A request is `verb;arg;arg...` and carries decimal numbers only, so the page
// never has to describe MoonBit types: it sends the parameters it has and reads
// the columns it gets back. A response is one record per line with the fields
// comma separated; a verb that has nothing to return answers a single `-`.

///|
/// Parse a decimal number: an optional sign, digits, and an optional fraction.
fn parse_number(text : String) -> Double {
  let chars = text.to_array()
  let n = chars.length()
  let mut i = 0
  let mut sign = 1.0
  if i < n && (chars[i] == '-' || chars[i] == '+') {
    if chars[i] == '-' {
      sign = -1.0
    }
    i = i + 1
  }
  let mut whole = 0.0
  while i < n && chars[i] >= '0' && chars[i] <= '9' {
    whole = whole * 10.0 + (chars[i].to_int() - 48).to_double()
    i = i + 1
  }
  let mut frac = 0.0
  let mut scale = 1.0
  if i < n && chars[i] == '.' {
    i = i + 1
    while i < n && chars[i] >= '0' && chars[i] <= '9' {
      frac = frac * 10.0 + (chars[i].to_int() - 48).to_double()
      scale = scale * 10.0
      i = i + 1
    }
  }
  sign * (whole + frac / scale)
}

///|
/// The `position`-th field of a request, or zero when the page omitted it.
fn field(parts : Array[String], position : Int32) -> Double {
  guard position < parts.length() else { return 0.0 }
  parse_number(parts[position])
}

///|
/// The `position`-th field of a request as a cell index.
fn field_index(parts : Array[String], position : Int32) -> Int32 {
  field(parts, position).to_int()
}

///|
/// Round a value to six decimals and drop the sign of a zero, so the page shows
/// a short column instead of the full binary expansion.
fn clean(value : Double) -> Double {
  let rounded = (value * 1.0e6).round() / 1.0e6
  if rounded == 0.0 {
    0.0
  } else {
    rounded
  }
}

///|
/// One record: the fields comma separated.
fn record(values : Array[Double]) -> String {
  values.map(fn(v) { clean(v).to_string() }).join(",")
}

///|
/// A response body from its records.
fn table(rows : Array[String]) -> String {
  if rows.length() == 0 {
    "-"
  } else {
    rows.join("\n")
  }
}

///|
/// Split the underscore separated index list the page sends for blocked cells.
fn index_list(text : String) -> Array[Int32] {
  if text == "" || text == "-" {
    return []
  }
  let out : Array[Int32] = []
  for part in text.split("_") {
    out.push(parse_number(part.to_owned()).to_int())
  }
  out
}
