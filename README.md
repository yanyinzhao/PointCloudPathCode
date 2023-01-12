# Path Oracle on Point Cloud

## Overview

This project provides the implementation of the algorithm for calculating a shortest path oracle on point cloud.

We divide the comparison algorithms into two types, i.e., (1) our algorithm / oracle that the calculated path passes on point cloud and the algorithm / oracle that the calculated path passes on the implicit terrain surface, i.e., path on point cloud & terrain surface, and (2) our algorithm / oracle that the calculated path passes on point cloud and the algorithm / oracle that the calculated path passes on vertex of the implicit terrain surface, i.e., path on point cloud & vertex of terrain surface. 

- For the first type, our oracle CO, our on-the-fly algorithm CF, and the baselines, i.e., TTE-SEO, TTA-SEO, C-SEO, TTEO-N, TTAO-N, CO-N, TTEO, TTAO, TFTE, and TFTA are studied in the experiments. We include C-SEO, TTEO-N, TTAO-N, CO-N, TTEO, and TTAO for ablation test.

- For the second type, our oracle CO, our on-the-fly algorithm CF, and the baselines, i.e., TV-SEO, C-SEO, TVO-N, CO-N, TVO, and TFV are studied in the experiments. We include TV-SEO, C-SEO, TVO-N, CO-N, and TVO for ablation test. 

We refer the readers to our paper for more details.

In total, we compared 16 algorithms as follows:

- TTE-SEO (oracle based baseline)
- TTA-SEO (oracle based baseline)
- TV-SEO (variation oracle)
- C-SEO (variation oracle)
- TTEO-N (variation oracle)
- TTAO-N (variation oracle)
- TVO-N (variation oracle)
- CO-N (variation oracle)
- TTEO (variation oracle)
- TTAO (variation oracle)
- TVO (variation oracle)
- CO (our oracle)
- TFTE (on-the-fly baseline)
- TFTA (on-the-fly baseline)
- TFV (variation on-the-fly)
- CF (our on-the-fly)

Make sure there is a folder called "input/" and a folder called "output/" under the working directory. They will be used for storing the input/output files.

The source code are stored in "src/" folder.

## Dataset

The dataset are stored in "input/" folder.

The datasets are as follows:

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
- "RM_10092.xyz" (small version default resolution RM point cloud dataset with 10092 points)
- "RM_500208.xyz" (large version default resolution BH point cloud dataset with 500208 points)
- "RM_1000518.xyz" (large version multiresolution BH point cloud dataset with 1000518 points)
- "RM_1501668.xyz" (large version multiresolution BH point cloud dataset with 1501668 points)
- "RM_2000832.xyz" (large version multiresolution BH point cloud dataset with 2000832 points)
- "RM_2502075.xyz" (large version multiresolution BH point cloud dataset with 2502075 points)

- "BH_50_poi_on_10086.txt" (POI list with POI number of 50 on "BH_10086.xyz")
- "BH_100_poi_on_10086.txt" (POI list with POI number of 100 on "BH_10086.xyz")
- "BH_150_poi_on_10086.txt" (POI list with POI number of 150 on "BH_10086.xyz")
- "BH_200_poi_on_10086.txt" (POI list with POI number of 200 on "BH_10086.xyz")
- "BH_250_poi_on_10086.txt" (POI list with POI number of 250 on "BH_10086.xyz")
- "BH_50_poi_on_500835.txt" (POI list with POI number of 50 on "BH_500835.xyz")
- "BH_50_poi_on_1000414.txt" (POI list with POI number of 50 on "BH_1000414.xyz")
- "BH_50_poi_on_1500996.txt" (POI list with POI number of 50 on "BH_1500996.xyz")
- "BH_50_poi_on_2001610.txt" (POI list with POI number of 50 on "BH_2001610.xyz")
- "BH_50_poi_on_2502596.txt" (POI list with POI number of 50 on "BH_2502596.xyz")
- "BH_100_poi_on_500835.txt" (POI list with POI number of 100 on "BH_500835.xyz")
- "BH_150_poi_on_500835.txt" (POI list with POI number of 150 on "BH_500835.xyz")
- "BH_200_poi_on_500835.txt" (POI list with POI number of 200 on "BH_500835.xyz")
- "BH_250_poi_on_500835.txt" (POI list with POI number of 250 on "BH_500835.xyz")
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
- "EP_50_poi_on_500384.txt" (POI list with POI number of 50 on "EP_500384.xyz")
- "EP_50_poi_on_1001040.txt" (POI list with POI number of 50 on "EP_1001040.xyz")
- "EP_50_poi_on_1501578.txt" (POI list with POI number of 50 on "EP_1501578.xyz")
- "EP_50_poi_on_2001536.txt" (POI list with POI number of 50 on "EP_2001536.xyz")
- "EP_50_poi_on_2500560.txt" (POI list with POI number of 50 on "EP_2500560.xyz")
- "EP_100_poi_on_500384.txt" (POI list with POI number of 100 on "EP_500384.xyz")
- "EP_150_poi_on_500384.txt" (POI list with POI number of 150 on "EP_500384.xyz")
- "EP_200_poi_on_500384.txt" (POI list with POI number of 200 on "EP_500384.xyz")
- "EP_250_poi_on_500384.txt" (POI list with POI number of 250 on "EP_500384.xyz")
- "EP_500_poi_on_500384.txt" (POI list with POI number of 500 on "EP_500384.xyz")
- "EP_500_poi_on_1001040.txt" (POI list with POI number of 500 on "EP_1001040.xyz")
- "EP_500_poi_on_1501578.txt" (POI list with POI number of 500 on "EP_1501578.xyz")
- "EP_500_poi_on_2001536.txt" (POI list with POI number of 500 on "EP_2001536.xyz")
- "EP_500_poi_on_2500560.txt" (POI list with POI number of 500 on "EP_2500560.xyz")
- "EP_1000_poi_on_500384.txt" (POI list with POI number of 1000 on "EP_500384.xyz")
- "EP_1500_poi_on_500384.txt" (POI list with POI number of 1500 on "EP_500384.xyz")
- "EP_2000_poi_on_500384.txt" (POI list with POI number of 2000 on "EP_500384.xyz")
- "EP_2500_poi_on_500384.txt" (POI list with POI number of 2500 on "EP_500384.xyz")
- "RM_50_poi_on_10092.txt" (POI list with POI number of 50 on "RM_10092.xyz")
- "RM_100_poi_on_10092.txt" (POI list with POI number of 100 on "RM_10092.xyz")
- "RM_150_poi_on_10092.txt" (POI list with POI number of 150 on "RM_10092.xyz")
- "RM_200_poi_on_10092.txt" (POI list with POI number of 200 on "RM_10092.xyz")
- "RM_250_poi_on_10092.txt" (POI list with POI number of 250 on "RM_10092.xyz")
- "RM_50_poi_on_500208.txt" (POI list with POI number of 50 on "RM_500208.xyz")
- "RM_50_poi_on_1000518.txt" (POI list with POI number of 50 on "RM_1000518.xyz")
- "RM_50_poi_on_1501668.txt" (POI list with POI number of 50 on "RM_1501668.xyz")
- "RM_50_poi_on_2000832.txt" (POI list with POI number of 50 on "RM_2000832.xyz")
- "RM_50_poi_on_2502075.txt" (POI list with POI number of 50 on "RM_2502075.xyz")
- "RM_100_poi_on_500208.txt" (POI list with POI number of 100 on "RM_500208.xyz")
- "RM_150_poi_on_500208.txt" (POI list with POI number of 150 on "RM_500208.xyz")
- "RM_200_poi_on_500208.txt" (POI list with POI number of 200 on "RM_500208.xyz")
- "RM_250_poi_on_500208.txt" (POI list with POI number of 250 on "RM_500208.xyz")
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

## Compile command

```
cd src
g++ -o main main.cpp -std=c++11
```

## Run command

```
./main [point_cloud_data_and_point_number_and_poi_number_map_index] [epsilon] [run_knn]
```

The meaning for each parameter is as follows:

- [point_cloud_data_and_point_number_and_poi_number_map_index]: an index for the map of point cloud data and dataset size and poi number (a integer from 0 to 72)
- [epsilon]: the epsilon value (0 < epsilon <= 1)
- [run_knn]: whether to run all POIs knn query (0 means not running knn query, 1 means running knn query)

For the [point_cloud_data_and_point_number_and_poi_number_map_index], each index value corresponding to a point cloud data, the dataset size of the point cloud and the poi number on the terrain, their relationships are as follows:

| Index | Point cloud data | Point number | POI number |
| ----------- | ----------- | ----------- | ----------- |
| 0 | BH | 10086 | 50 |
| 1 | BH | 10086 | 100 |
| 2 | BH | 10086 | 150 |
| 3 | BH | 10086 | 200 |
| 4 | BH | 10086 | 250 |
| 5 | EP | 10062 | 50 |
| 6 | EP | 10062 | 100 |
| 7 | EP | 10062 | 150 |
| 8 | EP | 10062 | 200 |
| 9 | EP | 10062 | 250 |
| 10 | EP | 20130 | 50 |
| 11 | EP | 30098 | 50 |
| 12 | EP | 40076 | 50 |
| 13 | EP | 50373 | 50 |
| 14 | RM | 10092 | 50 |
| 15 | RM | 10092 | 100 |
| 16 | RM | 10092 | 150 |
| 17 | RM | 10092 | 200 |
| 18 | RM | 10092 | 250 |
| 19 | BH | 500835 | 50 |
| 20 | BH | 500835 | 100 |
| 21 | BH | 500835 | 150 |
| 22 | BH | 500835 | 200 |
| 23 | BH | 500835 | 250 |
| 24 | BH | 1000414 | 50 |
| 25 | BH | 1500996 | 50 |
| 26 | BH | 2001610 | 50 |
| 27 | BH | 2502596 | 50 |
| 28 | EP | 500384 | 50 |
| 29 | EP | 500384 | 100 |
| 30 | EP | 500384 | 150 |
| 31 | EP | 500384 | 200 |
| 32 | EP | 500384 | 250 |
| 33 | EP | 1001040 | 50 |
| 34 | EP | 1501578 | 50 |
| 35 | EP | 2001536 | 50 |
| 36 | EP | 2500560 | 50 |
| 37 | RM | 500208 | 50 |
| 38 | RM | 500208 | 100 |
| 39 | RM | 500208 | 150 |
| 40 | RM | 500208 | 200 |
| 41 | RM | 500208 | 250 |
| 42 | RM | 1000518 | 50 |
| 43 | RM | 1501668 | 50 |
| 44 | RM | 2000832 | 50 |
| 45 | RM | 2502075 | 50 |
| 46 | BH | 500835 | 500 |
| 47 | BH | 500835 | 1000 |
| 48 | BH | 500835 | 1500 |
| 49 | BH | 500835 | 2000 |
| 50 | BH | 500835 | 2500 |
| 51 | BH | 1000414 | 500 |
| 52 | BH | 1500996 | 500 |
| 53 | BH | 2001610 | 500 |
| 54 | BH | 2502596 | 500 |
| 55 | EP | 500384 | 500 |
| 56 | EP | 500384 | 1000 |
| 57 | EP | 500384 | 1500 |
| 58 | EP | 500384 | 2000 |
| 59 | EP | 500384 | 2500 |
| 60 | EP | 1001040 | 500 |
| 61 | EP | 1501578 | 500 |
| 62 | EP | 2001536 | 500 |
| 63 | EP | 2500560 | 500 |
| 64 | RM | 500208 | 500 |
| 65 | RM | 500208 | 1000 |
| 66 | RM | 500208 | 1500 |
| 67 | RM | 500208 | 2000 |
| 68 | RM | 500208 | 2500 |
| 69 | RM | 1000518 | 500 |
| 70 | RM | 1501668 | 500 |
| 71 | RM | 2000832 | 500 |
| 72 | RM | 2502075 | 500 |

As mentioned in our paper, TTE-SEO, TTA-SEO, TTEO-N, TTAO-N are very time consuming (they are only feasible on small-version dataset), and TV-SEO, C-SEO, TVO-N, CO-N are also time consuming (they are only feasible on small-verion dataset and large-version dataset with small-version POIs). So the project will run TTE-SEO, TTA-SEO, TV-SEO, C-SEO, TTEO-N, TTAO-N, TVO-N, CO-N, TTEO, TTAO, TVO, CO, TFTE, TFTA, TFV and CF on small-version dataset ([point_cloud_data_and_point_number_and_poi_number_map_index] <= 18). The project will run TV-SEO, C-SEO, TVO-N, CO-N, TTEO, TTAO, TVO, CO, TFTE, TFTA, TFV and CF on large-version dataset with small-version POIs (18 < [point_cloud_data_and_point_number_and_poi_number_map_index] <= 45). The project will run TTEO, TTAO, TVO, CO, TFTE, TFTA, TFV and CF on large-version dataset with large-version POIs ([point_cloud_data_and_point_number_and_poi_number_map_index] > 45).

In addition, we strongly encourage you to set [run_knn] to 0 if you are not conducting experiments. Otherwise, it will take a very long time to run calculate the knn of all POIs. 

An example:

```
./main 0 0.5 0
```

In this example, [point_cloud_data_and_point_number_and_poi_number_map_index] is 0, [epsilon] is 0.5, [run_knn] is 0. So, it will run BH point cloud dataset, with point number equal to 10086 and poi number equal to 50, epsilon is 0.5, and it will not run all POIs knn query. It will run 16 algorithms, i.e., TTE-SEO, TTA-SEO, TV-SEO, C-SEO, TTEO-N, TTAO-N, TVO-N, CO-N, TTEO, TTAO, TVO, CO, TFTE, TFTA, TFV and CF.

## Output

The output will be stored in "output/output.txt" file. The format will be as follows:

```

[dataset] [point_num] [poi_num] [epsilon] [point_cloud_to_terrain_time (ms)] [construction_time (ms)] [query_time (ms)] [point_cloud_to_terrain_memroy_usage (MB)] [memory_usage (MB)] [index_size (MB)] [distance_error_point_cloud] [distance_error_terrain] [knn_query_time] [knn_error_point_cloud] [knn_error_terrain]

```

These information will also be shown in the terminal. 