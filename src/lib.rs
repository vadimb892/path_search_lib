#[ allow( dead_code ) ]

use std::collections::LinkedList;
use std::collections::BinaryHeap;
use std::collections::HashMap;
use core::hash::Hash;
use std::cmp::Ordering;
use std::fmt::Debug;
use std::cmp::Reverse;
use rand::{ Rng, distributions::Uniform };
use pancurses::*;

pub trait Network< T >
{
  fn neighbors( &self, node : &T ) -> Vec< T >;
}

pub trait WeightedNetwork< T > : Network< T >
{
  fn cost( &self, from : &T, to : &T ) -> f64;
}

pub struct WeightedGraph< T >
{
  edges : HashMap< T, Vec< T > >,
  weights : HashMap< ( T, T ), f64 >
}

impl< T : Hash + Eq + Copy > Network< T > for WeightedGraph< T >
{
  fn neighbors( &self, node: &T ) -> Vec< T >
  {
    self.edges[ &node ].clone( )
  }
}

impl< T : Hash + Eq + Copy > WeightedNetwork< T > for WeightedGraph< T >
{
  fn cost( &self, from : &T, to : &T ) -> f64
  {
    self.weights[ &( from.clone( ), to.clone( ) ) ]
  }
}

#[ derive( Eq, PartialEq, PartialOrd, Ord, Hash, Copy, Clone, Debug ) ]
pub struct Point
{
  pub x : i32,
  pub y : i32
}

impl Point
{
  pub fn new( x : i32, y : i32 ) -> Point
  {
    Point{ x, y }
  }

  fn heuristic( &self, to : &Point ) -> f64
  {
    let x2 = f64::powf( f64::abs( ( self.x - to.x ).into( ) ), 2.0 );
    let y2 = f64::powf( f64::abs( ( self.y - to.y ).into( ) ), 2.0 );
    f64::sqrt( x2 + y2 )
  }

  fn projection( &self, rectangle : ( Point, Point ) ) -> Option< Point >
  {
    let ( Point{ x : x0, y : y0 }, Point{ x : x1, y : y1 } ) = rectangle;
    if !( x0 <= self.x && self.x < x1 && y0 <= self.y && self.y < y1 )
    {
      return None
    }
    let p_x = self.x - x0;
    let p_y = self.y - y0; 
    Some( Point::new( p_x, p_y ) )
  }
}

pub struct WeightedGrid
{
  height : u32,
  width : u32,
  mode : Mode,
  weights : Vec< Vec< f64 > >,
  walls : Vec< Vec< bool > >
}

fn from( number : i32 ) -> usize
{
  number.try_into( ).unwrap( )
}

fn flat_matrix< T : Clone >( width : u32, height : u32, filler : T ) 
-> Vec< Vec< T > >
{
  let w : usize = from( width as i32 );
  let h : usize = from( height as i32 );
  vec![ vec![ filler; w ]; h ]
}

#[ inline ]
pub fn random_matrix( width : u32, height : u32 ) 
-> Vec< Vec< f64 > >
{
  let rng = rand::thread_rng( );
  let distribution = Uniform::from( 0.0..20.0 );
  let mut weights = Vec::< Vec< f64 > >::new( );
  for _ in 0..height 
  {
    weights.push( 
    rng.clone( )
    .sample_iter( &distribution )
    .take( from( width as i32 ) )
    .collect( ) );
  }
  weights.clone( )
}

#[ derive( Clone ) ]
pub enum Mode
{
  Flat,
  Random
}

fn gen_default( mode : Mode ) -> f64
{
  match mode
  {
    Mode::Flat => 1.0,
    Mode::Random => rand::thread_rng( ).gen_range( 0.0..20.0 ),
    _ => 1.0,
  }
}

impl WeightedGrid
{
  #[ inline ]
  pub fn new( width : u32, height : u32, mode : Mode ) -> WeightedGrid
  {
    let weights = match mode
    {
      Mode::Flat => flat_matrix::< f64 >( width, height, 1.0 ),
      Mode::Random => random_matrix( width, height ),
      _ => flat_matrix::< f64 >( width, height, 1.0 ),
    };
    WeightedGrid
    {
      height,
      width,
      mode,
      weights,
      walls : flat_matrix::< bool >( width, height, false )
    }
  }
  
  fn in_bounds( &self, id : &Point ) -> bool 
  {
    0 <= id.x &&
    0 <= id.y &&
    self.width as i32 > id.x &&
    self.height as i32 > id.y
  }

  fn is_wall( &self, id : &Point ) -> bool
  {
    self.walls[ from( id.y ) ][ from( id.x ) ] || 
    self.weights[ from( id.y ) ][ from( id.x ) ].is_infinite( )
  }
}

impl Network< Point > for WeightedGrid
{
  fn neighbors( &self, node : &Point ) -> Vec< Point >
  {
    if self.is_wall( node )
    {
      return vec![];
    }
    let Point{ x, y } = node;
    let neighbors = vec![
      Point::new( x + 1, *y ),
      Point::new( x - 1, *y ),
      Point::new( *x, y + 1 ),
      Point::new( *x, y - 1 ), 
      Point::new( x - 1, y - 1 ),
      Point::new( x - 1, y + 1 ),
      Point::new( x + 1, y - 1 ),
      Point::new( x + 1, y + 1 )
    ];
    let n = neighbors.into_iter( )
    .filter( | id | self.in_bounds( id ) )
    .filter( | id | !self.is_wall( id ) )
    .collect::< Vec< _ > >( );
    n
  }
}

impl WeightedNetwork< Point > for WeightedGrid
{
  fn cost( &self, from : &Point, to : &Point ) -> f64
  {
    let neighbors = self.neighbors( from );
    if !neighbors.contains( &to )
    {
      return f64::INFINITY;
    }
    self.weights[ to.y as usize ][ to.x as usize ]
  }
}

impl Clone for WeightedGrid
{
  fn clone( &self ) -> Self
  {
    Self
    {
      height : self.height,
      width : self.width,
      mode : self.mode.clone( ),
      weights : self.weights.clone( ),
      walls : self.walls.clone( )
    }
  }
}

#[ derive( Clone ) ]
pub struct DistanceTo< T >( pub T, pub f64 );

impl< T > Eq for DistanceTo< T > { }

impl< T > Ord for DistanceTo< T >
{
  fn cmp( &self, other : &DistanceTo< T > ) -> Ordering 
  {
    if self.1 < other.1
    {
      return Ordering::Less;
    }
    else if self.1 > other.1
    {
      return Ordering::Greater;
    }
    else
    {
      return Ordering::Equal;
    }
  }
}

impl< T > PartialOrd for DistanceTo< T > 
{
  fn partial_cmp( &self, other : &DistanceTo< T > ) -> Option< Ordering > 
  {
    if self.1 < other.1
    {
      return Some( Ordering::Less );
    }
    else if self.1 > other.1
    {
      return Some( Ordering::Greater );
    }
    else
    {
      return Some( Ordering::Equal );
    }
  }
}

impl< T > PartialEq for DistanceTo< T > 
{
  fn eq( &self, other : &DistanceTo< T > ) -> bool 
  {
    self.1 == other.1
  }
}

pub struct FIFOAlgorithState< T >
{
  start : T, 
  end : T, 
  completed : bool,
  frontier : LinkedList::< T >,
  came_from : HashMap::< T, Option< T > >
}

impl< T : Eq + PartialEq + Hash + Copy > FIFOAlgorithState< T >
{
  #[ inline ]
  pub fn new( start : T, end : T ) -> FIFOAlgorithState< T >
  {
    let mut frontier = LinkedList::< T >::new( );
    frontier.push_front( start );
    let mut came_from = HashMap::< T, Option< T > >::new( );
    came_from.insert( start, None );
    FIFOAlgorithState::< T >
    {
      start,
      end,
      completed : false,
      frontier,
      came_from
    }
  } 
}

pub struct PriorityAlgorithState< T >
{
  start : T, 
  end : T, 
  completed : bool,
  frontier : BinaryHeap::< Reverse < DistanceTo::< T > > >,
  came_from : HashMap::< T, Option< T > >,
  cost_so_far : HashMap::< T, f64 >
}

impl< T : Eq + PartialEq + Hash + Copy > PriorityAlgorithState< T >
{
  #[ inline ]
  pub fn new( start : T, end : T ) -> PriorityAlgorithState< T >
  {
    let mut frontier = BinaryHeap::< Reverse< DistanceTo< T > > >::new( );
    frontier.push( Reverse ( DistanceTo::< T >( start, 0.0 ) ) );
    let mut came_from = HashMap::< T, Option< T > >::new( );
    let mut cost_so_far = HashMap::< T, f64 >::new( );
    came_from.insert( start, None );
    cost_so_far.insert( start, 0.0 );
    PriorityAlgorithState::< T >
    { 
      start,
      end,
      completed : false,
      frontier,
      came_from,
      cost_so_far
    }
  }
}

#[ inline ]
pub fn breadth_first_search< T >
( 
  state : &mut FIFOAlgorithState< T >, 
  network : &dyn Network< T > 
) 
-> Option< T >
where T: Eq + Copy + Hash + Debug
{
  if state.frontier.is_empty( ) || state.completed
  {
    return None
  }
  let current = state.frontier.pop_back( ).unwrap( );
  if current == state.end 
  {
    state.completed = true;
    return None
  }
  for next in network.neighbors( &current )
  {
    if !state.came_from.contains_key( &next )
    {
      state.frontier.push_front( next );
      state.came_from.insert( next, Some( current ) );
    }
  }
  Some( current )
}

#[ inline ]
pub fn dijkstra_search< T >
( 
  state : &mut PriorityAlgorithState< T >, 
  network : &dyn WeightedNetwork< T > 
) 
-> Option< T >
where T: Eq + Copy + Hash
{
  if state.frontier.is_empty( ) || state.completed
  {
    return None
  }
  let current = state.frontier.pop( ).unwrap( ).0.0;
  if current == state.end
  {
    state.completed = true;
    return None
  } 
  for next in network.neighbors( &current )
  {
    let new_cost = state.cost_so_far[ &current ] + 
    network.cost( &current, &next );
    if !state.cost_so_far.contains_key( &next ) || new_cost < state.cost_so_far[ &next ]
    {
      state.cost_so_far.insert( next, new_cost );
      state.frontier.push( Reverse( DistanceTo( next, new_cost ) ) );
      state.came_from.insert( next, Some( current ) );
    }
  }
  Some( current )
}

#[ inline ]
pub fn a_star_search
( 
  state : &mut PriorityAlgorithState< Point >, 
  network : &dyn WeightedNetwork< Point > 
) 
-> Option< Point >
{
  if state.frontier.is_empty( ) || state.completed
  {
    return None
  }
  let current = state.frontier.pop( ).unwrap( ).0.0;
  if current == state.end
  {
    state.completed = true;
    return None
  } 
  for next in network.neighbors( &current )
  {
    let new_cost = state.cost_so_far[ &current ] + 
    network.cost( &current, &next );
    if !state.cost_so_far.contains_key( &next ) || new_cost < state.cost_so_far[ &next ]
    {
      state.cost_so_far.insert( next, new_cost );
      state.frontier.push
      ( 
        Reverse (
          DistanceTo::< Point >
          ( 
            next.clone( ), 
            new_cost + next.heuristic( &state.end ) 
          ) 
        )
      );
      state.came_from.insert( next, Some( current ) );
    }
  }
  Some( current )
}

#[ derive( PartialEq ) ]
enum LoopState 
{
  Continue,
  End
}

struct TaskState
{
  network : WeightedGrid,
  start : Point,
  end : Point,
  current_iteration : u32,
  bfs_state : FIFOAlgorithState::< Point >,
  d_state : PriorityAlgorithState::< Point >,
  a_state : PriorityAlgorithState::< Point >
}

impl TaskState 
{
  fn new( network : WeightedGrid, start : Point, end : Point ) -> TaskState
  {
    let bfs_state = FIFOAlgorithState::< Point >::new( start, end );
    let d_state = PriorityAlgorithState::< Point >::new( start, end );
    let a_state = PriorityAlgorithState::< Point >::new( start, end );	
    TaskState
    {
      network,
      start, 
      end, 
      current_iteration : 0, 
      bfs_state, 
      d_state, 
      a_state, 
    }
  }
}

struct DrawingState
{
  maps_position : Vec< (Point, Point) >,
}

impl DrawingState
{
  fn new( width : u32, height : u32, offset : Point ) -> DrawingState
  {
    let mut drawing_state = DrawingState 
    { 
      maps_position: Vec::< (Point, Point) >::new( ),
    };

    let mut last_offset = offset.x;
    for _ in 0..3
    {
      let begin_p = Point
      { 
        x : last_offset, 
        y : offset.y
      };
      last_offset += width as i32;
      let end_p = Point
      { 
        x : last_offset, 
        y : offset.y + height as i32
      }; 
      last_offset += 3;
      drawing_state.maps_position.push( ( begin_p, end_p ) );
    }
    drawing_state
  }
}

struct InputState
{
  mouse_events : HashMap< mmask_t, MEVENT >
}

enum AlgorithmState< 'a, T >
{
  FIFOState( &'a FIFOAlgorithState< T > ),
  PriorityState( &'a PriorityAlgorithState< T > )
}

pub struct Testbench
{
  window : Window,
  task_state : TaskState,
  drawing_state : DrawingState,
  input_state : InputState,
  loop_state : LoopState,
}

impl Testbench
{
  #[ inline ]
  pub fn new( network : WeightedGrid ) -> Testbench
  {
    let offset = Point{ x : 3, y : 2 };
    let start = Point::new( 0, 0 );
    let end = Point::new( 0, 0 );
    let window = initscr( );

    window.keypad( true ); 
    mousemask( ALL_MOUSE_EVENTS, Some( &mut 0 ) ); 

    window.refresh( );
    let task_state = TaskState::new( network, start, end );
    let drawing_state = DrawingState::new
    ( 
      task_state.network.width, 
      task_state.network.height, 
      offset 
    );
    let input_state = 
    InputState
    { 
      mouse_events : HashMap::< mmask_t, MEVENT >::new( ) 
    };
    Testbench 
    { 
      window, 
      task_state,
      drawing_state,
      input_state,
      loop_state : LoopState::Continue 
    }
  }

  fn end_loop( &mut self )
  {
    self.loop_state = LoopState::End;
  }

  fn run( &mut self )
  {
    self.redraw( );
    while self.loop_state != LoopState::End 
    { 
      self.handle_input( );
    }
    self.end_curses( );
  }

  fn turn( &mut self )
  {
    breadth_first_search( &mut self.task_state.bfs_state, &self.task_state.network );
    dijkstra_search( &mut self.task_state.d_state, &self.task_state.network );
    a_star_search( &mut self.task_state.a_state, &self.task_state.network );
    self.task_state.current_iteration += 1;
  } 

  fn draw_map( &self )
  {
    let w = self.task_state.network.width;
    let h = self.task_state.network.height;
    let walls = &self.task_state.network.walls;
    let mut current_char : char;
    for p in &self.drawing_state.maps_position
    {
      for i in 0..h
      {
        for j in 0..w
        {
          if !walls[ i as usize ][ j as usize ]
          {
            current_char = ',';
          } 
          else
          {
            current_char = '#';
          }
          let x = p.0.x + j as i32;
          let y = p.0.y + i as i32;
          self.window.mvaddch( y, x, current_char );
        }
      }
    } 
  }

  fn draw_frontier( &self, rect : (Point, Point), frontier : Vec< Point >, current : Point )
  {
    for i in frontier
    {
      let x = rect.0.x + i.x;
      let y = rect.0.y + i.y;
      self.window.mvaddch( y, x, 'x' );
    }
    let x = rect.0.x + current.x;
    let y = rect.0.y + current.y;
    self.window.mvaddch( y, x, '@' );
  }

  fn draw_ends( &self, rect : (Point, Point) )
  {
    let x = rect.0.x + self.task_state.start.x;
    let y = rect.0.y + self.task_state.start.y;
    self.window.mvaddch( y, x, 's' );

    let x = rect.0.x + self.task_state.end.x;
    let y = rect.0.y + self.task_state.end.y;
    self.window.mvaddch( y, x, 'e' );
  }

  fn draw_path( &self, rect : (Point, Point), path : Vec< Point > )
  {
    for i in path
    {
      let x = rect.0.x + i.x;
      let y = rect.0.y + i.y;
      self.window.mvaddch( y, x, 'o' );
    }
  }

  fn draw_cost( &self, cost : f64, map_position : ( Point, Point ) )
  {
    let x = map_position.0.x;
    let y = map_position.1.y + 1; 
    self.window.mvaddstr( y, x, format!("Cost: {:.5}", cost) );
  }

  fn get_path( &self, came_from : &HashMap< Point, Option< Point > > ) -> Vec< Point >
  {
    let mut path = Vec::< Point >::new( );
    let mut current = self.task_state.end;
    path.push( current );
    loop
    {
      current = match came_from[ &current ]
      {
        Some( current ) => 
        {
          path.push( current );
          current
        },
        None => break
      }
    };
    path.reverse( );
    path.clone( )
  }

  fn draw_process( &self, state_enum : AlgorithmState< Point >, map_position : usize ) 
  {
    let rect = self.drawing_state.maps_position[ map_position ];
    match state_enum
    {
      AlgorithmState::FIFOState( state ) => 
      {
        if !state.completed
        {
          let frontier = state.frontier.clone( )
          .into_iter( )
          .collect::< Vec< Point > >( );
          self.draw_frontier( rect, frontier, state.frontier.back( ).unwrap( ).clone( ) );
        }
        else
        { 
          let path = self.get_path( &state.came_from );
          self.draw_path( rect, path.clone( ) );
          self.draw_cost( ( path.len( ) - 1 ) as f64, rect );
        }
      },
      AlgorithmState::PriorityState( state ) => 
      {
        if !state.completed
        {
          let frontier = state.frontier.clone( )
          .into_iter( )
          .map( | d | d.0.0 )
          .collect::< Vec< Point > >( );
          self.draw_frontier( rect, frontier, state.frontier.peek( ).unwrap( ).0.0 );
        }
        else
        { 
          let path = self.get_path( &state.came_from );
          let cost = state.cost_so_far[ &self.task_state.end ];
          self.draw_path( rect, path.clone( ) );
          self.draw_cost( cost, rect );
        }
      },
      _ => ( )
    };
    self.draw_ends( rect );
  }

  #[ inline ]
  pub fn redraw( &self )
  {
    let t = &self.task_state;
    self.window.clear( );		
    self.draw_map( );
    self.draw_process( AlgorithmState::FIFOState( &t.bfs_state ), 0 );
    self.draw_process( AlgorithmState::PriorityState( &t.d_state ), 1 );
    self.draw_process( AlgorithmState::PriorityState( &t.a_state ), 2 );
    self.window.refresh( );
  }

  fn reset( &mut self )
  {
    self.task_state = TaskState::new( 
      self.task_state.network.clone( ),
      self.task_state.start,
      self.task_state.end
    );
  }

  fn set_start( &mut self, start : Point )
  {
    self.task_state.start = start;
    self.task_state.bfs_state.start = start; 
    self.task_state.d_state.start = start; 
    self.task_state.a_state.start = start; 
  }

  fn set_end( &mut self, end : Point )
  {
    self.task_state.end = end;
    self.task_state.bfs_state.end = end; 
    self.task_state.d_state.end = end; 
    self.task_state.a_state.end = end; 
  }

  fn handle_input( &mut self )
  {
    match self.window.getch( ) {
      Some( Input::KeyMouse ) => {
          if let Ok( mouse_event ) = getmouse( ) {
            self.handle_mouse( mouse_event )
          };
          self.process_mouse( );
      }
      Some( Input::Character( x ) ) => self.handle_keyboard( x ),
      _ => ( ),
    }		
  }

  fn handle_keyboard( &mut self, character : char )
  {
    match character
    {
      'r' => 
      {
        self.reset( );
        self.redraw( );
      },
      'w' => 
      {
        self.turn( );
        self.redraw( );
      },
      'q' => self.end_loop( ),
      _ => ( ),
    }
  }

  fn process_mouse( &mut self )
  {
    let events = self.input_state.mouse_events.clone( );
    let event_keys = Vec::from_iter( events.keys( ) );
    for key in event_keys
    {
      let event = self.input_state.mouse_events[ key ];
      let p = Point{ x : event.x, y : event.y };
      match *key
      {
        BUTTON1_CLICKED => 
        {
          let mode = self.task_state.network.mode.clone( );
          let walls = &mut self.task_state.network.walls;
          let weights = &mut self.task_state.network.weights;
          if walls[ from( p.y ) ][ from( p.x ) ] == false
          {
            weights[ from( p.y ) ][ from( p.x ) ] = f64::INFINITY;
          }
          else
          {
            weights[ from( p.y ) ][ from( p.x ) ] = gen_default( mode );
          }
          walls[ from( p.y ) ][ from( p.x ) ] = !walls[ from( p.y ) ][ from( p.x ) ];
          self.reset( ); 
          self.redraw( );
          self.input_state.mouse_events.remove( &key ); 
        },
        BUTTON2_CLICKED => 
        {
          self.set_start( p );
          self.reset( );
          self.redraw( ); 
          self.input_state.mouse_events.remove( &key ); 
        },
        BUTTON3_CLICKED => 
        {
          self.set_end( p );
          self.reset( );
          self.redraw( );
          self.input_state.mouse_events.remove( &key ); 
        },
        _ => { self.input_state.mouse_events.remove( &key ); },
      }
    }
  }

  fn handle_mouse( &mut self, mut event : MEVENT )
  {  
    let origin_p = Point{ x : event.x, y : event.y };
    let mut hit = false;
    for begin_p in &self.drawing_state.maps_position 
    {
      if let Some(local_p) = origin_p.projection(*begin_p )
      {
        event.x = local_p.x;
        event.y = local_p.y;
        hit = true;
      }
    }
    if !hit
    {
      return
    }
    self.input_state.mouse_events.insert( event.bstate, event );
  }

  fn end_curses( &self )
  {
    endwin();
  }
}