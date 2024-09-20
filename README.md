# Proximity Queries on Point Clouds using Rapid Construction Path Oracle

## Overview

This project provides the implementation for proximity queries on point cloud using rapid construction path oracle. We refer the readers to our paper for more details.

We compared 25 algorithms as follows (the algorithms calculate the path passing on a point cloud as default):

- SE-Oracle-Adapt (oracle based baseline)
- SE-Oracle-Adapt-A2A (oracle based baseline for A2A query)
- SE-Oracle-FastFly-Adapt (oracle in ablation study)
- SE-Oracle-FastFly-Adapt-A2A (oracle in ablation study for A2A query)
- SE-Oracle (oracle based baseline that calculates the path passing on a TIN)
- EAR-Oracle-Adapt (oracle based baseline)
- EAR-Oracle-FastFly-Adapt (oracle in ablation study)
- EAR-Oracle (oracle based baseline that calculates the path passing on a TIN)
- SU-Oracle-Adapt (oracle for other proximity queries)
- RC-Oracle-Naive (variation oracle)
- RC-Oracle-Naive-A2A (variation oracle for A2A query)
- RC-Oracle-NaiveProx (our oracle with the naive proximity queries algorithm)
- RC-Oracle-NaiveProx-A2A (our oracle with the naive proximity queries algorithm for A2A query)
- RC-Oracle-Naive-Adapt (variation oracle that calculates the path passing on a TIN)
- RC-Oracle (our oracle with the efficient proximity queries algorithm)
- RC-Oracle-A2A (our oracle with the efficient proximity queries algorithm for A2A query)
- RC-Oracle-Adapt (our oracle with the efficient proximity queries algorithm that calculate the path passing on a TIN)
- CH-Adapt (on-the-fly baseline algorithm)
- CH (on-the-fly baseline algorithm that calculates the path passing on a TIN)
- Kaul-Adapt (on-the-fly baseline algorithm)
- Kaul (on-the-fly baseline algorithm that calculates the path passing on a TIN)
- Dijk-Adapt (on-the-fly baseline algorithm)
- Dijk (on-the-fly baseline algorithm that calculates the path passing on a TIN)
- FastFly (our on-the-fly algorithm)
- FastFly-Adapt (our on-the-fly algorithm that calculate the path passing on a TIN)

## Environment: 
- Linux machine (2.2 GHz CPU and 512GB memory)
- g++
- gnuplot (for plotting paper graphs)

## Reproducible run

We provide simple commands to run all experiments and plot all paper graphs (including the graphs in our technical report). Running experiments for each one figure takes 2—3 days, and running experiments for all figures takes 3—4 months. We also provide the sample experiments data used for plot all paper graphs (in the case that you do not have enough time to run the experiments). We also provide simple examples to test all algorithms (takes 10 minutes), please see section called "normal run" below. Before you run, please download some of the datasets from https://drive.google.com/drive/folders/1_Sblnq9XOLFwvPI2lX_l8KJJzVWR828y?usp=drive_link, and save in the "input/" folder, since these datasets are very large, and they exceed the maximum file upload size in Github. 

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

- "BH_1014" (small version default resolution BH point cloud or TIN dataset with 1014 points or vertices)
- "BH_10086" (small version default resolution BH point cloud or TIN dataset with 10086 points or vertices)
- "BH_500835" (large version default resolution BH point cloud or TIN dataset with 500835 points or vertices)
- "BH_1000414" (large version multiresolution BH point cloud or TIN dataset with 1000414 points or vertices)
- "BH_1500996" (large version multiresolution BH point cloud or TIN dataset with 1500996 points or vertices)
- "BH_2001610" (large version multiresolution BH point cloud or TIN dataset with 2001610 points or vertices)
- "BH_2502596" (large version multiresolution BH point cloud or TIN dataset with 2502596 points or vertices)
- "EP_10062" (small version default resolution EP point cloud or TIN dataset with 10062 points or vertices)
- "EP_20130" (small version multiresolution EP point cloud or TIN dataset with 20130 points or vertices)
- "EP_30098" (small version multiresolution EP point cloud or TIN dataset with 30098 points or vertices)
- "EP_40076" (small version multiresolution EP point cloud or TIN dataset with 40076 points or vertices)
- "EP_50373" (small version multiresolution EP point cloud or TIN dataset with 50373 points or vertices)
- "EP_500384" (large version default resolution EP point cloud or TIN dataset with 500384 points or vertices)
- "EP_1001040" (large version multiresolution EP point cloud or TIN dataset with 1001040 points or vertices)
- "EP_1501578" (large version multiresolution EP point cloud or TIN dataset with 1501578 points or vertices)
- "EP_2001536" (large version multiresolution EP point cloud or TIN dataset with 2001536 points or vertices)
- "EP_2500560" (large version multiresolution EP point cloud or TIN dataset with 2500560 points or vertices)
- "GF_10092" (small version default resolution GF point cloud or TIN dataset with 10092 points or vertices)
- "GF_500208" (large version default resolution GF point cloud or TIN dataset with 500208 points or vertices)
- "GF_1000518" (large version multiresolution GF point cloud or TIN dataset with 1000518 points or vertices)
- "GF_1501668" (large version multiresolution GF point cloud or TIN dataset with 1501668 points or vertices)
- "GF_2000832" (large version multiresolution GF point cloud or TIN dataset with 2000832 points or vertices)
- "GF_2502075" (large version multiresolution GF point cloud or TIN dataset with 2502075 points or vertices)
- "LM_10092" (small version default resolution LM point cloud or TIN dataset with 10092 points or vertices)
- "LM_500208" (large version default resolution LM point cloud or TIN dataset with 500208 points or vertices)
- "LM_1000518" (large version multiresolution LM point cloud or TIN dataset with 1000518 points or vertices)
- "LM_1501668" (large version multiresolution LM point cloud or TIN dataset with 1501668 points or vertices)
- "LM_2000832" (large version multiresolution LM point cloud or TIN dataset with 2000832 points or vertices)
- "LM_2502075" (large version multiresolution LM point cloud or TIN dataset with 2502075 points or vertices)
- "RM_10092" (small version default resolution RM point cloud or TIN dataset with 10092 points or vertices)
- "RM_500208" (large version default resolution RM point cloud or TIN dataset with 500208 points or vertices)
- "RM_1000518" (large version multiresolution RM point cloud or TIN dataset with 1000518 points or vertices)
- "RM_1501668" (large version multiresolution RM point cloud or TIN dataset with 1501668 points or vertices)
- "RM_2000832" (large version multiresolution RM point cloud or TIN dataset with 2000832 points or vertices)
- "RM_2502075" (large version multiresolution RM point cloud or TIN dataset with 2502075 points or vertices)

- "BH_50_poi_on_1014.txt" (POI list with POI number of 50 on "BH_1014")
- "BH_50_poi_on_10086.txt" (POI list with POI number of 50 on "BH_10086")
- "BH_100_poi_on_10086.txt" (POI list with POI number of 100 on "BH_10086")
- "BH_150_poi_on_10086.txt" (POI list with POI number of 150 on "BH_10086")
- "BH_200_poi_on_10086.txt" (POI list with POI number of 200 on "BH_10086")
- "BH_250_poi_on_10086.txt" (POI list with POI number of 250 on "BH_10086")
- "BH_500_poi_on_500835.txt" (POI list with POI number of 500 on "BH_500835")
- "BH_500_poi_on_1000414.txt" (POI list with POI number of 500 on "BH_1000414")
- "BH_500_poi_on_1500996.txt" (POI list with POI number of 500 on "BH_1500996")
- "BH_500_poi_on_2001610.txt" (POI list with POI number of 500 on "BH_2001610")
- "BH_500_poi_on_2502596.txt" (POI list with POI number of 500 on "BH_2502596")
- "BH_1000_poi_on_500835.txt" (POI list with POI number of 1000 on "BH_500835")
- "BH_1500_poi_on_500835.txt" (POI list with POI number of 1500 on "BH_500835")
- "BH_2000_poi_on_500835.txt" (POI list with POI number of 2000 on "BH_500835")
- "BH_2500_poi_on_500835.txt" (POI list with POI number of 2500 on "BH_500835")
- "EP_50_poi_on_10062.txt" (POI list with POI number of 50 on "EP_10062")
- "EP_50_poi_on_20130.txt" (POI list with POI number of 50 on "EP_20130")
- "EP_50_poi_on_30098.txt" (POI list with POI number of 50 on "EP_30098")
- "EP_50_poi_on_40076.txt" (POI list with POI number of 50 on "EP_40076")
- "EP_50_poi_on_50373.txt" (POI list with POI number of 50 on "EP_50373")
- "EP_100_poi_on_10062.txt" (POI list with POI number of 100 on "EP_10062")
- "EP_150_poi_on_10062.txt" (POI list with POI number of 150 on "EP_10062")
- "EP_200_poi_on_10062.txt" (POI list with POI number of 200 on "EP_10062")
- "EP_250_poi_on_10062.txt" (POI list with POI number of 250 on "EP_10062")
- "EP_500_poi_on_500384.txt" (POI list with POI number of 500 on "EP_500384")
- "EP_500_poi_on_1001040.txt" (POI list with POI number of 500 on "EP_1001040")
- "EP_500_poi_on_1501578.txt" (POI list with POI number of 500 on "EP_1501578")
- "EP_500_poi_on_2001536.txt" (POI list with POI number of 500 on "EP_2001536")
- "EP_500_poi_on_2500560.txt" (POI list with POI number of 500 on "EP_2500560")
- "EP_1000_poi_on_500384.txt" (POI list with POI number of 1000 on "EP_500384")
- "EP_1500_poi_on_500384.txt" (POI list with POI number of 1500 on "EP_500384")
- "EP_2000_poi_on_500384.txt" (POI list with POI number of 2000 on "EP_500384")
- "EP_2500_poi_on_500384.txt" (POI list with POI number of 2500 on "EP_500384")
- "GF_50_poi_on_10092.txt" (POI list with POI number of 50 on "GF_10092")
- "GF_100_poi_on_10092.txt" (POI list with POI number of 100 on "GF_10092")
- "GF_150_poi_on_10092.txt" (POI list with POI number of 150 on "GF_10092")
- "GF_200_poi_on_10092.txt" (POI list with POI number of 200 on "GF_10092")
- "GF_250_poi_on_10092.txt" (POI list with POI number of 250 on "GF_10092")
- "GF_500_poi_on_500208.txt" (POI list with POI number of 500 on "GF_500208")
- "GF_500_poi_on_1000518.txt" (POI list with POI number of 500 on "GF_1000518")
- "GF_500_poi_on_1501668.txt" (POI list with POI number of 500 on "GF_1501668")
- "GF_500_poi_on_2000832.txt" (POI list with POI number of 500 on "GF_2000832")
- "GF_500_poi_on_2502075.txt" (POI list with POI number of 500 on "GF_2502075")
- "GF_1000_poi_on_500208.txt" (POI list with POI number of 1000 on "GF_500208")
- "GF_1500_poi_on_500208.txt" (POI list with POI number of 1500 on "GF_500208")
- "GF_2000_poi_on_500208.txt" (POI list with POI number of 2000 on "GF_500208")
- "GF_2500_poi_on_500208.txt" (POI list with POI number of 2500 on "GF_500208")
- "LM_50_poi_on_10092.txt" (POI list with POI number of 50 on "LM_10092")
- "LM_100_poi_on_10092.txt" (POI list with POI number of 100 on "LM_10092")
- "LM_150_poi_on_10092.txt" (POI list with POI number of 150 on "LM_10092")
- "LM_200_poi_on_10092.txt" (POI list with POI number of 200 on "LM_10092")
- "LM_250_poi_on_10092.txt" (POI list with POI number of 250 on "LM_10092")
- "LM_500_poi_on_500208.txt" (POI list with POI number of 500 on "LM_500208")
- "LM_500_poi_on_1000518.txt" (POI list with POI number of 500 on "LM_1000518")
- "LM_500_poi_on_1501668.txt" (POI list with POI number of 500 on "LM_1501668")
- "LM_500_poi_on_2000832.txt" (POI list with POI number of 500 on "LM_2000832")
- "LM_500_poi_on_2502075.txt" (POI list with POI number of 500 on "LM_2502075")
- "LM_1000_poi_on_500208.txt" (POI list with POI number of 1000 on "LM_500208")
- "LM_1500_poi_on_500208.txt" (POI list with POI number of 1500 on "LM_500208")
- "LM_2000_poi_on_500208.txt" (POI list with POI number of 2000 on "LM_500208")
- "LM_2500_poi_on_500208.txt" (POI list with POI number of 2500 on "LM_500208")
- "RM_50_poi_on_10092.txt" (POI list with POI number of 50 on "RM_10092")
- "RM_100_poi_on_10092.txt" (POI list with POI number of 100 on "RM_10092")
- "RM_150_poi_on_10092.txt" (POI list with POI number of 150 on "RM_10092")
- "RM_200_poi_on_10092.txt" (POI list with POI number of 200 on "RM_10092")
- "RM_250_poi_on_10092.txt" (POI list with POI number of 250 on "RM_10092")
- "RM_500_poi_on_500208.txt" (POI list with POI number of 500 on "RM_500208")
- "RM_500_poi_on_1000518.txt" (POI list with POI number of 500 on "RM_1000518")
- "RM_500_poi_on_1501668.txt" (POI list with POI number of 500 on "RM_1501668")
- "RM_500_poi_on_2000832.txt" (POI list with POI number of 500 on "RM_2000832")
- "RM_500_poi_on_2502075.txt" (POI list with POI number of 500 on "RM_2502075")
- "RM_1000_poi_on_500208.txt" (POI list with POI number of 1000 on "RM_500208")
- "RM_1500_poi_on_500208.txt" (POI list with POI number of 1500 on "RM_500208")
- "RM_2000_poi_on_500208.txt" (POI list with POI number of 2000 on "RM_500208")
- "RM_2500_poi_on_500208.txt" (POI list with POI number of 2500 on "RM_500208")

Data Format:

For the point cloud dataset, we used the .xyz format in the experiment. The content of the .xyz file is as follows:

```
points_num

1st_point_x_coord 1st_point_y_coord 1st_point_z_coord

2nd_point_x_coord 2nd_point_y_coord 2nd_point_z_coord

......

last_point_x_coord last_point_y_coord last_point_z_coord
```

For the TIN dataset, we used the .off format in the experiment. The content of the .off file is as follows:

```
OFF

vertices_num faces_num edges_num

1st_vertex_x_coord 1st_vertex_y_coord 1st_vertex_z_coord

2nd_vertex_x_coord 2nd_vertex_y_coord 2nd_vertex_z_coord

......

last_vertex_x_coord last_vertex_y_coord last_vertex_z_coord

1st_face_1st_vertex_ID 1st_face_2nd_vertex_ID 1st_face_3td_vertex_ID

2nd_face_1st_vertex_ID 2nd_face_2nd_vertex_ID 2nd_face_3td_vertex_ID

......

last_face_1st_vertex_ID last_face_2nd_vertex_ID last_face_3td_vertex_ID
```

For the POI list, we used the .txt format in the experiment. The content of the .txt file is as follows:

```
POI_num
1st_POI_index 2nd_POI_index ......
```


## Normal run 

### Compile command

Before you run, please download some of the datasets from https://drive.google.com/drive/folders/1_Sblnq9XOLFwvPI2lX_l8KJJzVWR828y?usp=drive_link, and save in the "input/" folder, since these datasets are very large, and they exceed the maximum file upload size in Github. The source code are stored in "src/" folder.

```
cd src
g++ -o main main.cpp -std=c++11
```

### Run command

```
./main [data_and_dataset_size_and_poi_number_map_index] [epsilon] [run_knn_query] [run_range_query]
```

The meaning for each parameter is as follows:

- [data_and_dataset_size_and_poi_number_map_index]: an index for the map of point cloud or TIN data and dataset size and poi number (a integer from 0 to 119)
- [epsilon]: the epsilon value (0 < epsilon <= 1)
- [run_knn_query]: whether to run all POIs knn query (0 means not running knn query, 1 means running knn query)
- [run_range_query]: whether to run all POIs range query (0 means not running range query, 1 means running range query)

For the [data_and_dataset_size_and_poi_number_map_index], each index value corresponding to a point cloud or TIN data (each point cloud and TIN dataset represent the same region), the dataset size and the poi number, their relationships are as follows:

| Index | Point cloud or TIN data | Dataset size | POI number |
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

Since SE-Oracle-Adapt, EAR-Oracle-Adapt, SE-Oracle, EAR-Oracle, SU-Oracle-Adapt, RC-Oracle-Naive, RC-Oracle-Naive-Adapt, SE-Oracle-Adapt-A2A, and RC-Oracle-Naive-A2A are time consuming, the project will run all algorithms on small-version dataset with default 50 POIs([data_and_dataset_size_and_poi_number_map_index] <= 29). The project will run all algorithms except the 8 mentioned algorithm on large-version dataset with default 500 POIs ([data_and_dataset_size_and_poi_number_map_index] > 29).

In addition, we strongly encourage you to set [run_knn_query] and [run_range_query] to 0 if you are not conducting experiments. Otherwise, it will take a very long time to calculate them. 

An example:

```
./main 0 0.5 0 0
```

In this example, [data_and_dataset_size_and_poi_number_map_index] is 0, [epsilon] is 0.5, [run_knn_query] is 0, [run_range_query] is 0. So, it will run BH point cloud and TIN dataset, with dataset size equal to 10086 and poi number equal to 50, epsilon is 0.5, it will not run all POIs knn query and will not run all POIs range query. It will run all algorithms.

Make sure there is a folder called "input/" and a folder called "output/" under the working directory. They will be used for storing the input/output files.

### Output

The output will be stored in "output/output.txt" file. The format will be as follows:

```
[dataset] [dataset_size] [poi_num] [epsilon] [point_cloud_to_terrain_time (ms)] [construction_time (ms)] [query_time (ms)] [point_cloud_to_terrain_memroy_usage (MB)] [memory_usage (MB)] [output_size (MB)] [distance_error_point_cloud] [distance_error_terrain] [knn_query_time] [knn_error_point_cloud] [knn_error_terrain] [range_query_time] [range_error_point_cloud] [range_error_terrain]
```

These information will also be shown in the terminal. 