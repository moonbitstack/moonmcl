// A minimal binary min-heap of (priority, cell) entries. It is a stand-in for
// the general-purpose priority queue so the search keeps a strictly-smallest
// pop order without depending on a `Compare` impl for a custom type; the
// tie-break on the cell makes the pop order deterministic.

///|
pub struct MinHeap {
  data : Array[(Int32, Int32)]
}

///|
pub fn MinHeap::min_heap() -> MinHeap {
  { data: [], }
}

///|
fn less(a : (Int32, Int32), b : (Int32, Int32)) -> Bool {
  if a.0 != b.0 {
    a.0 < b.0
  } else {
    a.1 < b.1
  }
}

///|
pub fn MinHeap::push(self : MinHeap, priority : Int32, cell : Int32) -> Unit {
  self.data.push((priority, cell))
  let mut i = self.data.length() - 1
  while i > 0 {
    let p = (i - 1) / 2
    if less(self.data[i], self.data[p]) {
      let t = self.data[i]
      self.data[i] = self.data[p]
      self.data[p] = t
      i = p
    } else {
      break
    }
  }
}

///|
/// Pop and return the smallest entry, or `None` when empty.
pub fn MinHeap::pop(self : MinHeap) -> (Int32, Int32)? {
  guard self.data is [top, ..] else { return None }
  let last = self.data[self.data.length() - 1]
  self.data.truncate(self.data.length() - 1)
  if self.data.length() > 0 {
    self.data[0] = last
    let mut i = 0
    let n = self.data.length()
    while true {
      let l = 2 * i + 1
      let r = 2 * i + 2
      let mut smallest = i
      if l < n && less(self.data[l], self.data[smallest]) {
        smallest = l
      }
      if r < n && less(self.data[r], self.data[smallest]) {
        smallest = r
      }
      if smallest == i {
        break
      }
      let t = self.data[i]
      self.data[i] = self.data[smallest]
      self.data[smallest] = t
      i = smallest
    }
  }
  Some(top)
}

///|
pub fn MinHeap::is_empty(self : MinHeap) -> Bool {
  self.data.is_empty()
}
