use criterion::*;
use pancurses::*;
use path_search_lib::*;
use rand::Rng;
use std::cmp::Reverse;
use std::collections::BinaryHeap;

//ncurses symbol update speed bench
fn addch_benchmark( c : &mut Criterion ) 
{
  let mut rng = rand::thread_rng( );
  const CHARSET: &[ u8 ] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZ\
                          abcdefghijklmnopqrstuvwxyz\
                          0123456789)(*&^%$#@!~";
  let set_size = CHARSET.len( );
  let mut window = initscr( );
  window.resize( 80, 60 );

  c.bench_function( 
    "addch", 
    | b | b.iter( 
      | | window.mvaddch( 
        black_box( rng.gen_range( 0..10 ) ), 
        black_box( rng.gen_range( 0..10 ) ), 
        black_box( CHARSET[ rng.gen_range( 0..set_size ) ] as char ) 
      )
    )
  );
  endwin( );
}

//redraw speed
fn redraw_benchmark( c : &mut Criterion ) 
{
  let ( w, h ) = ( 20, 20 );
  let network = WeightedGrid::new( w, h, Mode::Random );
  let environment = Testbench::new( network );

  c.bench_function( 
    "redraw_speed", 
    | b | b.iter( 
      | | environment.redraw( )
    )
  );
}

//redraw speed dependency from map size
fn redraw_size_dependency_benchmark( c : &mut Criterion ) 
{
  let mut group = c.benchmark_group( "redraw_size_dependency_group" );
  for size in ( 8..=40 ).step_by( 8 )
  {
    let network = WeightedGrid::new( size, size, Mode::Random );
    let environment = Testbench::new( network );
    group.bench_function( 
      format!( "redraw_map_size_dependency_{size}" ),
      | b | b.iter( | | environment.redraw( ) )
    );
  }
  group.finish( );
}

//BFS, Dijkstra, A* turn speed
fn search_benchmark( c : &mut Criterion )
{
  let mut group = c.benchmark_group( "search_group" );
  let start = Point::new( 0, 0 );
  let end = Point::new( 35, 35 );
  let mut bfs_state = FIFOAlgorithState::< Point >::new( start, end );
  let mut d_state = PriorityAlgorithState::< Point >::new( start, end );
  let mut a_state = PriorityAlgorithState::< Point >::new( start, end );
  let network = WeightedGrid::new( 40, 40, Mode::Random );

  group.bench_function( 
    "BFS_speed", 
    | b | b.iter( 
      | | breadth_first_search( 
        black_box( &mut bfs_state ), 
        black_box( &network )   
      )
    )
  );

  group.bench_function( 
    "Dijkstra_speed", 
    | b | b.iter( 
      | | dijkstra_search( 
        black_box( &mut d_state ), 
        black_box( &network )   
      )
    )
  );

  group.bench_function( 
    "A*_speed", 
    | b | b.iter( 
      | | a_star_search( 
        black_box( &mut a_state ), 
        black_box( &network )   
      )
    )
  );

  group.finish( );
}

//BFS Dijkstra A* speed dependency from path length
fn search_size_dependency_benchmark( c : &mut Criterion )
{
  let mut group = c.benchmark_group( "search_size_dependency_group" );
  for size in ( 8..=40 ).step_by( 8 )
  {
    let start = Point::new( 0, 0 );
    let end = Point::new( size - 1, size - 1 );
    let mut bfs_state = FIFOAlgorithState::< Point >::new( start, end );
    let mut d_state = PriorityAlgorithState::< Point >::new( start, end );
    let mut a_state = PriorityAlgorithState::< Point >::new( start, end );
    let s : u32 = size.try_into( ).unwrap( );
    let network = WeightedGrid::new( s, s, Mode::Random );
    let sample_size : usize = ( ( s * s ) / 2 ).try_into( ).unwrap( );
    group.significance_level( 0.1 ).sample_size( sample_size );
    group.bench_function( 
      format!( "BFS_speed_map_size_dependency_{size}" ), 
      | b | b.iter( 
        | | breadth_first_search( 
          black_box( &mut bfs_state ), 
          black_box( &network )   
        )
      )
    );

    group.bench_function( 
      format!( "Dijkstra_speed_map_size_dependency_{size}" ), 
      | b | b.iter( 
        | | dijkstra_search( 
          black_box( &mut d_state ), 
          black_box( &network )   
        )
      )
    );

    group.bench_function( 
      format!( "A*_speed_map_size_dependency_{size}" ), 
      | b | b.iter( 
        | | a_star_search( 
          black_box( &mut a_state ), 
          black_box( &network )   
        )
      )
    );
  }	
  group.finish( );
}

//random_matrix speed
fn random_matrix_benchmark( c : &mut Criterion ) 
{
  c.bench_function( 
    "random_matrix_speed", 
    | b | b.iter( 
      | | random_matrix( 
        black_box( 50 ), 
        black_box( 50 )   
      )
    )
  );
}

//random_matrix speed dependency from map size
fn random_matrix_size_dependency_benchmark( c : &mut Criterion ) 
{
  let mut group = c.benchmark_group( "random_matrix_size_dependency_group" );
  for size in ( 8..=40 ).step_by( 8 )
  {
    group.bench_with_input( 
      BenchmarkId::new( format!("random_matrix_size_dependency_{size}"), size ), 
      &size,
      | b, &size | b.iter( | | random_matrix( size, size ) )
    );
  }
  group.finish( );
}

//BinaryHeap DistanceTo insert speed
fn binary_heap_benchmark( c : &mut Criterion ) 
{
  let mut rng = rand::thread_rng( );
  let mut heap = BinaryHeap::< Reverse< DistanceTo< Point > > >::new( );
  c.bench_function( 
    "binary_heap_insert_speed", 
    | b | b.iter( 
      | | 
      {
        let ( x, y ) = ( rng.gen_range( 0..10 ), rng.gen_range( 0..10 ) );
        let d = rng.gen_range( 0..1000 );
        let input = Reverse ( DistanceTo::< Point >( Point::new( x, y ), d as f64 ) );
        heap.push( black_box( input ) ) 
      }
    )
  );
}

criterion_group!{
  name = addch_group;
  config = Criterion::default().significance_level( 0.1 ).sample_size( 500 );
  targets = addch_benchmark
}

criterion_group!{
    name = redraw_group;
    config = Criterion::default().significance_level( 0.1 ).sample_size( 500 );
    targets = redraw_benchmark
}

criterion_group!{
  name = redraw_size_dependency_group;
  config = Criterion::default().significance_level( 0.1 ).sample_size( 100 );
  targets = redraw_size_dependency_benchmark
}

criterion_group!{
  name = search_group;
  config = Criterion::default().significance_level( 0.1 ).sample_size( 1000 );
  targets = search_benchmark
}

criterion_group!(search_size_dependency_group, search_size_dependency_benchmark);

criterion_group!{
  name = random_matrix_group;
  config = Criterion::default().significance_level( 0.1 ).sample_size( 1000 );
  targets = random_matrix_benchmark
}

criterion_group!{
  name = random_matrix_size_dependency_group;
  config = Criterion::default().significance_level( 0.1 ).sample_size( 100 );
  targets = random_matrix_size_dependency_benchmark
}

criterion_group!{
  name = binary_heap_group;
  config = Criterion::default().significance_level( 0.1 ).sample_size( 10000 );
  targets = binary_heap_benchmark
}
 
criterion_main!(
  addch_group, 
  redraw_group, 
  redraw_size_dependency_group,
  search_group,
  search_size_dependency_group,
  random_matrix_group,
  random_matrix_size_dependency_group, 
  binary_heap_group
);