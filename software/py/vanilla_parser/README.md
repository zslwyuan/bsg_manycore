 # Vanilla Parser

 Package of following Python modules to parse logs generated by BSG Manycore's simulation. Typically, all that's needed to generate essential information is to run:
 
 ```
 $ python3 -m vanilla_parser
 ```
 
 If default Python search path doesn't include this directory, you might have to run:
 ```
 $ PYTHONPATH=<path to this dir>/.. python3 -m vanilla_parser
 ```
 

A guide for using these parsers is located here: [HammerBlade Cosimulation Profiling Guide](https://docs.google.com/presentation/d/1ICDw5ZYETJ8DNcvNaR2vFLoJpP_GdNW_eUedG1_ISso/edit)
 
 
 ## Vanilla Parser accepts following logs files generated by simulation:
 - `--trace`: default filename is `vanilla_operation_trace.csv`
 - `--stats`: default filename is `vanilla_stats.csv`
 - `--log`: default filename is `vanilla.log`
 - `--vcache-trace`: default filename is `vcache_operation_trace.csv`
 - `--vcache-stats`: defualt filename is `vcache_stats.csv`

 ## Per module input files:
 
 ```
 - blood_graph: trace and stats
 - stats_parser: stats
 - pc_histogram: trace
 - trace_parser: log
 - vcache_stall_graph: vcache-trace and vcache-stats 
 ```
 
 ## Full list of available options:

 ```
 $ python3 -m vanilla_parser -h
 
 usage: vanilla_parser [-h] [--trace TRACE] [--stats STATS] [--log LOG]
                       [--vcache-trace VCACHE_TRACE]
                       [--vcache-stats VCACHE_STATS] [--tile TILE]
                       [--tile-group TILE_GROUP] [--abstract] [--generate-key]
                       [--cycle CYCLE] [--only [SUBMODULE [SUBMODULE ...]]]
                       [--also [SUBMODULE [SUBMODULE ...]]] [--no-blood-graph]
                       [--per-vcache] [--no-stall-graph]

 The interface for parsing COSIM output logs. This parser comprises
 of following five tools:

 - blood_graph
 - stats_parser
 - pc_histogram
 - trace_parser
 - vcache_stall_graph

 By default blood_graph, stats_parser and pc_histogram are executed,
 given required files are generated by cosim. If required files for
 a tool are missing it is skipped.

 optional arguments:
   -h, --help            show this help message and exit
   --trace TRACE         Vanilla operation log file
   --stats STATS         Vanilla stats log file
   --log LOG             Vanilla log file
   --vcache-trace VCACHE_TRACE
                         Vanilla operation log file
   --vcache-stats VCACHE_STATS
                         Vanilla stats log file
   --tile TILE           Also generate per tile stats
   --tile-group TILE_GROUP
                         Also generate per tile group stats
   --abstract            Type of graphs - abstract / detailed
   --generate-key        Generate a key image with graphs
   --cycle CYCLE         Cycle window of bloodgraph/stallgraph as
                         start_cycle@end_cycle.
   --only [SUBMODULE [SUBMODULE ...]]
                         List of tools to run instead of the default set
   --also [SUBMODULE [SUBMODULE ...]]
                         List of tools to run in addition to the default set

 Blood graph specific options:
   --no-blood-graph      Skip blood graph generation

 Stats parser specific options:
   --per-vcache          Also generate separate stats files for each victim
                         cache bank.

 Vcache stall graph specific options:
   --no-stall-graph      Skip stall graph generation
 ```
