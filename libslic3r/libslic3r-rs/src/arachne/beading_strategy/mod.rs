//! arachne/beading_strategy module
//!
//! Auto-generated module declaration for arachne/beading_strategy

pub mod beading_strategy;
pub mod beading_strategy_factory;
pub mod distributed_beading_strategy;
pub mod limited_beading_strategy;
pub mod outer_wall_contour_strategy;
pub mod outer_wall_inset_beading_strategy;
pub mod redistribute_beading_strategy;
pub mod widening_beading_strategy;

// Re-export key types
pub use beading_strategy_factory::BeadingStrategyFactory;
pub use distributed_beading_strategy::DistributedBeadingStrategy;
pub use limited_beading_strategy::LimitedBeadingStrategy;
pub use outer_wall_inset_beading_strategy::OuterWallInsetBeadingStrategy;
pub use redistribute_beading_strategy::RedistributeBeadingStrategy;
pub use widening_beading_strategy::WideningBeadingStrategy;

// TODO: Port these from C++:
// - CenterDeviationBeadingStrategy
// - InwardDistributedBeadingStrategy
