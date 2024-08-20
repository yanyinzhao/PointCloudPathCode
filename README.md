# Proximity Queries on Point Clouds using Rapid Construction Path Oracle

## Overview

This project provides the implementation for proximity queries on point cloud using rapid construction path oracle. We refer the readers to our paper for more details.

We compared 12 algorithms as follows:

- SE-Oracle-Adapt (oracle based baseline)
- SE-Oracle-FastFly-Adapt (oracle in ablation study)
- EAR-Oracle-Adapt (oracle based baseline)
- EAR-Oracle-FastFly-Adapt (oracle in ablation study)
- SU-Oracle-Adapt (oracle for other proximity queries)
- RC-Oracle-Naive (variation oracle)
- RC-Oracle-NaiveProx (our oracle with the naive proximity queries algorithm)
- RC-Oracle (our oracle with the efficient proximity queries algorithm)
- CH-Adapt (on-the-fly baseline)
- Kaul-Adapt (on-the-fly baseline)
- Dijk-Adapt (on-the-fly baseline)
- FastFly (our on-the-fly)

## Environment: 
- Linux machine (2.2 GHz CPU and 512GB memory)
- g++
- gnuplot (for plotting paper graphs)

## Reproducible run

We provide simple commands to run all experiments and plot all paper graphs. Running experiments for each one figure takes 2—3 days, and running experiments for all figures takes 2—3 months. We also provide the sample experiments data used for plot all paper graphs (in the case that you do not have enough time to run the experiments). We also provide simple examples to test all algorithms (takes 10 minutes), please see section called "normal run" below. 

### Run all experiments command

```
cd src
g++ -o experiment experiment.cpp -std=c++11
./experiment
```
All experimental results will be saved in “../exp”

### Plot paper graphs command
```
cd ../exp
sh plot.sh
```
All paper graphs will be saved in “../exp/eps”

If you do not have enough time to run the experiments, you can skip “run all experiments command”, and directly use “plot paper graphs command”, since we provide the sample experiments data used for plot all paper graphs in “../exp”. If you run the experiments, these sample data will be override. 

## Dataset

The dataset are stored in "input/" folder.

The datasets are as follows:

- "BH_1014.xyz" (small version default resolution BH point cloud dataset with 1014 points)
- "BH_10086.xyz" (small version default resolution BH point cloud dataset with 10086 points)
- "BH_500835.xyz" (large version default resolution BH point cloud dataset with 500835 points)
- "BH_1000414.xyz" (large version multiresolution BH point cloud dataset with 1000414 points)
- "BH_1500996.xyz" (large version multiresolution BH point cloud dataset with 1500996 points)
- "BH_2001610.xyz" (large version multiresolution BH point cloud dataset with 2001610 points)
- "BH_2502596.xyz" (large version multiresolution BH point cloud dataset with 2502596 points)
- "EP_10062.xyz" (small version default resolution EP point cloud dataset with 10062 points)
- "EP_20130.xyz" (small version multiresolution EP point cloud dataset with 20130 points)
- "EP_30098.xyz" (small version multiresolution EP point cloud dataset with 30098 points)
- "EP_40076.xyz" (small version multiresolution EP point cloud dataset with 40076 points)
- "EP_50373.xyz" (small version multiresolution EP point cloud dataset with 50373 points)
- "EP_500384.xyz" (large version default resolution EP point cloud dataset with 500384 points)
- "EP_1001040.xyz" (large version multiresolution EP point cloud dataset with 1001040 points)
- "EP_1501578.xyz" (large version multiresolution EP point cloud dataset with 1501578 points)
- "EP_2001536.xyz" (large version multiresolution EP point cloud dataset with 2001536 points)
- "EP_2500560.xyz" (large version multiresolution EP point cloud dataset with 2500560 points)
- "GF_10092.xyz" (small version default resolution GF point cloud dataset with 10092 points)
- "GF_500208.xyz" (large version default resolution GF point cloud dataset with 500208 points)
- "GF_1000518.xyz" (large version multiresolution GF point cloud dataset with 1000518 points)
- "GF_1501668.xyz" (large version multiresolution GF point cloud dataset with 1501668 points)
- "GF_2000832.xyz" (large version multiresolution GF point cloud dataset with 2000832 points)
- "GF_2502075.xyz" (large version multiresolution GF point cloud dataset with 2502075 points)
- "LM_10092.xyz" (small version default resolution LM point cloud dataset with 10092 points)
- "LM_500208.xyz" (large version default resolution LM point cloud dataset with 500208 points)
- "LM_1000518.xyz" (large version multiresolution LM point cloud dataset with 1000518 points)
- "LM_1501668.xyz" (large version multiresolution LM point cloud dataset with 1501668 points)
- "LM_2000832.xyz" (large version multiresolution LM point cloud dataset with 2000832 points)
- "LM_2502075.xyz" (large version multiresolution LM point cloud dataset with 2502075 points)
- "RM_10092.xyz" (small version default resolution RM point cloud dataset with 10092 points)
- "RM_500208.xyz" (large version default resolution RM point cloud dataset with 500208 points)
- "RM_1000518.xyz" (large version multiresolution RM point cloud dataset with 1000518 points)
- "RM_1501668.xyz" (large version multiresolution RM point cloud dataset with 1501668 points)
- "RM_2000832.xyz" (large version multiresolution RM point cloud dataset with 2000832 points)
- "RM_2502075.xyz" (large version multiresolution RM point cloud dataset with 2502075 points)

- "BH_50_poi_on_1014.txt" (POI list with POI number of 50 on "BH_1014.xyz")
- "BH_50_poi_on_10086.txt" (POI list with POI number of 50 on "BH_10086.xyz")
- "BH_100_poi_on_10086.txt" (POI list with POI number of 100 on "BH_10086.xyz")
- "BH_150_poi_on_10086.txt" (POI list with POI number of 150 on "BH_10086.xyz")
- "BH_200_poi_on_10086.txt" (POI list with POI number of 200 on "BH_10086.xyz")
- "BH_250_poi_on_10086.txt" (POI list with POI number of 250 on "BH_10086.xyz")
- "BH_500_poi_on_500835.txt" (POI list with POI number of 500 on "BH_500835.xyz")
- "BH_500_poi_on_1000414.txt" (POI list with POI number of 500 on "BH_1000414.xyz")
- "BH_500_poi_on_1500996.txt" (POI list with POI number of 500 on "BH_1500996.xyz")
- "BH_500_poi_on_2001610.txt" (POI list with POI number of 500 on "BH_2001610.xyz")
- "BH_500_poi_on_2502596.txt" (POI list with POI number of 500 on "BH_2502596.xyz")
- "BH_1000_poi_on_500835.txt" (POI list with POI number of 1000 on "BH_500835.xyz")
- "BH_1500_poi_on_500835.txt" (POI list with POI number of 1500 on "BH_500835.xyz")
- "BH_2000_poi_on_500835.txt" (POI list with POI number of 2000 on "BH_500835.xyz")
- "BH_2500_poi_on_500835.txt" (POI list with POI number of 2500 on "BH_500835.xyz")
- "EP_50_poi_on_10062.txt" (POI list with POI number of 50 on "EP_10062.xyz")
- "EP_50_poi_on_20130.txt" (POI list with POI number of 50 on "EP_20130.xyz")
- "EP_50_poi_on_30098.txt" (POI list with POI number of 50 on "EP_30098.xyz")
- "EP_50_poi_on_40076.txt" (POI list with POI number of 50 on "EP_40076.xyz")
- "EP_50_poi_on_50373.txt" (POI list with POI number of 50 on "EP_50373.xyz")
- "EP_100_poi_on_10062.txt" (POI list with POI number of 100 on "EP_10062.xyz")
- "EP_150_poi_on_10062.txt" (POI list with POI number of 150 on "EP_10062.xyz")
- "EP_200_poi_on_10062.txt" (POI list with POI number of 200 on "EP_10062.xyz")
- "EP_250_poi_on_10062.txt" (POI list with POI number of 250 on "EP_10062.xyz")
- "EP_500_poi_on_500384.txt" (POI list with POI number of 500 on "EP_500384.xyz")
- "EP_500_poi_on_1001040.txt" (POI list with POI number of 500 on "EP_1001040.xyz")
- "EP_500_poi_on_1501578.txt" (POI list with POI number of 500 on "EP_1501578.xyz")
- "EP_500_poi_on_2001536.txt" (POI list with POI number of 500 on "EP_2001536.xyz")
- "EP_500_poi_on_2500560.txt" (POI list with POI number of 500 on "EP_2500560.xyz")
- "EP_1000_poi_on_500384.txt" (POI list with POI number of 1000 on "EP_500384.xyz")
- "EP_1500_poi_on_500384.txt" (POI list with POI number of 1500 on "EP_500384.xyz")
- "EP_2000_poi_on_500384.txt" (POI list with POI number of 2000 on "EP_500384.xyz")
- "EP_2500_poi_on_500384.txt" (POI list with POI number of 2500 on "EP_500384.xyz")
- "GF_50_poi_on_10092.txt" (POI list with POI number of 50 on "GF_10092.xyz")
- "GF_100_poi_on_10092.txt" (POI list with POI number of 100 on "GF_10092.xyz")
- "GF_150_poi_on_10092.txt" (POI list with POI number of 150 on "GF_10092.xyz")
- "GF_200_poi_on_10092.txt" (POI list with POI number of 200 on "GF_10092.xyz")
- "GF_250_poi_on_10092.txt" (POI list with POI number of 250 on "GF_10092.xyz")
- "GF_500_poi_on_500208.txt" (POI list with POI number of 500 on "GF_500208.xyz")
- "GF_500_poi_on_1000518.txt" (POI list with POI number of 500 on "GF_1000518.xyz")
- "GF_500_poi_on_1501668.txt" (POI list with POI number of 500 on "GF_1501668.xyz")
- "GF_500_poi_on_2000832.txt" (POI list with POI number of 500 on "GF_2000832.xyz")
- "GF_500_poi_on_2502075.txt" (POI list with POI number of 500 on "GF_2502075.xyz")
- "GF_1000_poi_on_500208.txt" (POI list with POI number of 1000 on "GF_500208.xyz")
- "GF_1500_poi_on_500208.txt" (POI list with POI number of 1500 on "GF_500208.xyz")
- "GF_2000_poi_on_500208.txt" (POI list with POI number of 2000 on "GF_500208.xyz")
- "GF_2500_poi_on_500208.txt" (POI list with POI number of 2500 on "GF_500208.xyz")
- "LM_50_poi_on_10092.txt" (POI list with POI number of 50 on "LM_10092.xyz")
- "LM_100_poi_on_10092.txt" (POI list with POI number of 100 on "LM_10092.xyz")
- "LM_150_poi_on_10092.txt" (POI list with POI number of 150 on "LM_10092.xyz")
- "LM_200_poi_on_10092.txt" (POI list with POI number of 200 on "LM_10092.xyz")
- "LM_250_poi_on_10092.txt" (POI list with POI number of 250 on "LM_10092.xyz")
- "LM_500_poi_on_500208.txt" (POI list with POI number of 500 on "LM_500208.xyz")
- "LM_500_poi_on_1000518.txt" (POI list with POI number of 500 on "LM_1000518.xyz")
- "LM_500_poi_on_1501668.txt" (POI list with POI number of 500 on "LM_1501668.xyz")
- "LM_500_poi_on_2000832.txt" (POI list with POI number of 500 on "LM_2000832.xyz")
- "LM_500_poi_on_2502075.txt" (POI list with POI number of 500 on "LM_2502075.xyz")
- "LM_1000_poi_on_500208.txt" (POI list with POI number of 1000 on "LM_500208.xyz")
- "LM_1500_poi_on_500208.txt" (POI list with POI number of 1500 on "LM_500208.xyz")
- "LM_2000_poi_on_500208.txt" (POI list with POI number of 2000 on "LM_500208.xyz")
- "LM_2500_poi_on_500208.txt" (POI list with POI number of 2500 on "LM_500208.xyz")
- "RM_50_poi_on_10092.txt" (POI list with POI number of 50 on "RM_10092.xyz")
- "RM_100_poi_on_10092.txt" (POI list with POI number of 100 on "RM_10092.xyz")
- "RM_150_poi_on_10092.txt" (POI list with POI number of 150 on "RM_10092.xyz")
- "RM_200_poi_on_10092.txt" (POI list with POI number of 200 on "RM_10092.xyz")
- "RM_250_poi_on_10092.txt" (POI list with POI number of 250 on "RM_10092.xyz")
- "RM_500_poi_on_500208.txt" (POI list with POI number of 500 on "RM_500208.xyz")
- "RM_500_poi_on_1000518.txt" (POI list with POI number of 500 on "RM_1000518.xyz")
- "RM_500_poi_on_1501668.txt" (POI list with POI number of 500 on "RM_1501668.xyz")
- "RM_500_poi_on_2000832.txt" (POI list with POI number of 500 on "RM_2000832.xyz")
- "RM_500_poi_on_2502075.txt" (POI list with POI number of 500 on "RM_2502075.xyz")
- "RM_1000_poi_on_500208.txt" (POI list with POI number of 1000 on "RM_500208.xyz")
- "RM_1500_poi_on_500208.txt" (POI list with POI number of 1500 on "RM_500208.xyz")
- "RM_2000_poi_on_500208.txt" (POI list with POI number of 2000 on "RM_500208.xyz")
- "RM_2500_poi_on_500208.txt" (POI list with POI number of 2500 on "RM_500208.xyz")

Data Format:

For the point cloud dataset, we used the .xyz format in the experiment. The content of the .xyz file is as follows:

```
OFF

points_num

1st_point_x_coord 1st_point_y_coord 1st_point_z_coord

2nd_point_x_coord 2nd_point_y_coord 2nd_point_z_coord

......

last_point_x_coord last_point_y_coord last_point_z_coord
```

For the POI list, we used the .txt format in the experiment. The content of the .txt file is as follows:

```
POI_num
1st_POI_index 2nd_POI_index ......
```


## Normal run 

### Compile command

The source code are stored in "src/" folder.

```
cd src
g++ -o main main.cpp -std=c++11
```

### Run command

```
./main [point_cloud_data_and_point_number_and_poi_number_map_index] [epsilon] [run_knn_query] [run_range_query]
```

The meaning for each parameter is as follows:

- [point_cloud_data_and_point_number_and_poi_number_map_index]: an index for the map of point cloud data and dataset size and poi number (a integer from 0 to 119)
- [epsilon]: the epsilon value (0 < epsilon <= 1)
- [run_knn_query]: whether to run all POIs knn query (0 means not running knn query, 1 means running knn query)
- [run_range_query]: whether to run all POIs range query (0 means not running range query, 1 means running range query)

For the [point_cloud_data_and_point_number_and_poi_number_map_index], each index value corresponding to a point cloud data, the dataset size of the point cloud and the poi number on the terrain, their relationships are as follows:

| Index | Point cloud data | Point number | POI number |
| ----------- | ----------- | ----------- | ----------- |
| 0 | BH | 1014 | 50 |
| 1 | BH | 10086 | 50 |
| 2 | BH | 10086 | 100 |
| 3 | BH | 10086 | 150 |
| 4 | BH | 10086 | 200 |
| 5 | BH | 10086 | 250 |
| 6 | EP | 10062 | 50 |
| 7 | EP | 10062 | 100 |
| 8 | EP | 10062 | 150 |
| 9 | EP | 10062 | 200 |
| 10 | EP | 10062 | 250 |
| 11 | EP | 20130 | 50 |
| 12 | EP | 30098 | 50 |
| 13 | EP | 40076 | 50 |
| 14 | EP | 50373 | 50 |
| 15 | GF | 10092 | 50 |
| 16 | GF | 10092 | 100 |
| 17 | GF | 10092 | 150 |
| 18 | GF | 10092 | 200 |
| 19 | GF | 10092 | 250 |
| 20 | LM | 10092 | 50 |
| 21 | LM | 10092 | 100 |
| 22 | LM | 10092 | 150 |
| 23 | LM | 10092 | 200 |
| 24 | LM | 10092 | 250 |
| 25 | RM | 10092 | 50 |
| 26 | RM | 10092 | 100 |
| 27 | RM | 10092 | 150 |
| 28 | RM | 10092 | 200 |
| 29 | RM | 10092 | 250 |
| 30 | BH | 500835 | 500 |
| 31 | BH | 500835 | 1000 |
| 32 | BH | 500835 | 1500 |
| 33 | BH | 500835 | 2000 |
| 34 | BH | 500835 | 2500 |
| 35 | BH | 1000414 | 500 |
| 36 | BH | 1500996 | 500 |
| 37 | BH | 2001610 | 500 |
| 38 | BH | 2502596 | 500 |
| 39 | EP | 500384 | 500 |
| 40 | EP | 500384 | 1000 |
| 41 | EP | 500384 | 1500 |
| 42 | EP | 500384 | 2000 |
| 43 | EP | 500384 | 2500 |
| 44 | EP | 1001040 | 500 |
| 45 | EP | 1501578 | 500 |
| 46 | EP | 2001536 | 500 |
| 47 | EP | 2500560 | 500 |
| 48 | GF | 500208 | 500 |
| 49 | GF | 500208 | 1000 |
| 50 | GF | 500208 | 1500 |
| 51 | GF | 500208 | 2000 |
| 52 | GF | 500208 | 2500 |
| 53 | GF | 1000518 | 500 |
| 54 | GF | 1501668 | 500 |
| 55 | GF | 2000832 | 500 |
| 56 | GF | 2502075 | 500 |
| 57 | LM | 500208 | 500 |
| 58 | LM | 500208 | 1000 |
| 59 | LM | 500208 | 1500 |
| 60 | LM | 500208 | 2000 |
| 61 | LM | 500208 | 2500 |
| 62 | LM | 1000518 | 500 |
| 63 | LM | 1501668 | 500 |
| 64 | LM | 2000832 | 500 |
| 65 | LM | 2502075 | 500 |
| 66 | RM | 500208 | 500 |
| 67 | RM | 500208 | 1000 |
| 68 | RM | 500208 | 1500 |
| 69 | RM | 500208 | 2000 |
| 70 | RM | 500208 | 2500 |
| 71 | RM | 1000518 | 500 |
| 72 | RM | 1501668 | 500 |
| 73 | RM | 2000832 | 500 |
| 74 | RM | 2502075 | 500 |

Since SE-Oracle-Adapt, EAR-Oracle-Adapt, SU-Oracle-Adapt, and RC-Oracle-Naive are time consuming, the project will run SE-Oracle-Adapt, SE-Oracle-FastFly-Adapt, EAR-Oracle-Adapt, EAR-Oracle-FastFly-Adapt, SU-Oracle-Adapt, RC-Oracle-Naive, RC-Oracle-NaiveProx, RC-Oracle, CH-Adapt, Kaul-Adapt, Dijk-Adapt, and FastFly on small-version dataset with default 50 POIs([point_cloud_data_and_point_number_and_poi_number_map_index] <= 29). The project will run SE-Oracle-FastFly-Adapt, EAR-Oracle-FastFly-Adapt, RC-Oracle-NaiveProx, RC-Oracle, CH-Adapt, Kaul-Adapt, Dijk-Adapt, and FastFly on large-version dataset with default 500 POIs ([point_cloud_data_and_point_number_and_poi_number_map_index] > 29).

In addition, we strongly encourage you to set [run_knn] to 0 if you are not conducting experiments. Otherwise, it will take a very long time to run calculate the knn of all POIs. 

An example:

```
./main 0 0.5 0 0
```

In this example, [point_cloud_data_and_point_number_and_poi_number_map_index] is 0, [epsilon] is 0.5, [run_knn_query] is 0, [run_range_query] is 0. So, it will run BH point cloud dataset, with point number equal to 10086 and poi number equal to 50, epsilon is 0.5, it will not run all POIs knn query and will not run all POIs range query. It will run 10 algorithms.

Make sure there is a folder called "input/" and a folder called "output/" under the working directory. They will be used for storing the input/output files.

### Output

The output will be stored in "output/output.txt" file. The format will be as follows:

```

[dataset] [point_num] [poi_num] [epsilon] [point_cloud_to_terrain_time (ms)] [construction_time (ms)] [query_time (ms)] [point_cloud_to_terrain_memroy_usage (MB)] [memory_usage (MB)] [index_size (MB)] [distance_error_point_cloud] [distance_error_terrain] [knn_query_time] [knn_error_point_cloud] [knn_error_terrain] [range_query_time] [range_error_point_cloud] [range_error_terrain]

```

These information will also be shown in the terminal. 