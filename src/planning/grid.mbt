// A rectangular 2-D lattice of free and blocked cells. Cells are addressed by
// a flat index `r * cols + c`; movement between cells is 4-connected with unit
// cost.

///|
pub struct Grid {
  blocked : Array[Bool]
  rows : Int32
  cols : Int32
}

///|
/// Build a `rows x cols` grid. `blocked_cells` are the flat indices of the
/// blocked cells (all others are free).
pub fn Grid::grid(
  rows : Int32,
  cols : Int32,
  blocked_cells~ : Array[Int32]? = Some([]),
) -> Grid {
  let n = rows * cols
  let blocked = Array::makei(n, _ => false)
  for cell in blocked_cells.unwrap_or([]).iter() {
    blocked[cell] = true
  }
  { blocked, rows, cols, }
}

///|
pub fn Grid::rows_of(self : Grid) -> Int32 {
  self.rows
}

///|
pub fn Grid::cols_of(self : Grid) -> Int32 {
  self.cols
}

///|
/// The flat index of the cell at row `r`, column `c`.
pub fn Grid::index(self : Grid, r : Int32, c : Int32) -> Int32 {
  r * self.cols + c
}

///|
pub fn Grid::is_blocked(self : Grid, cell : Int32) -> Bool {
  self.blocked[cell]
}

///|
/// The in-bounds free neighbours of `cell`, in a fixed order (up, right, down,
/// left).
pub fn Grid::neighbors(self : Grid, cell : Int32) -> Array[Int32] {
  let r = cell / self.cols
  let c = cell % self.cols
  let out : Array[Int32] = Array(capacity=4)
  if r > 0 && !self.blocked[cell - self.cols] {
    out.push(cell - self.cols)
  }
  if c + 1 < self.cols && !self.blocked[cell + 1] {
    out.push(cell + 1)
  }
  if r + 1 < self.rows && !self.blocked[cell + self.cols] {
    out.push(cell + self.cols)
  }
  if c > 0 && !self.blocked[cell - 1] {
    out.push(cell - 1)
  }
  out
}
