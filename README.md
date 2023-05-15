# Proximity Path Queries on Point Cloud using Rapid Construction Path Oracle

## Overview

This project provides the implementation for proximity path queries on point cloud using rapid construction path oracle. We refer the readers to our paper for more details.

We compared 20 algorithms as follows:

- SE-Oracle(FaceExact) (oracle based baseline)
- SE-Oracle(FaceAppr) (oracle based baseline)
- SE-Oracle(Vertex) (variation oracle)
- SE-Oracle(Point) (variation oracle)
- SE-Oracle-Adapt(FaceExact) (oracle based baseline)
- SE-Oracle-Adapt(FaceAppr) (oracle based baseline)
- SE-Oracle-Adapt(Vertex) (variation oracle)
- SE-Oracle-Adapt(Point) (variation oracle)
- RC-Oracle-Naive(FaceExact) (variation oracle)
- RC-Oracle-Naive(FaceAppr) (variation oracle)
- RC-Oracle-Naive(Vertex) (variation oracle)
- RC-Oracle-Naive(Point) (variation oracle)
- RC-Oracle(FaceExact) (variation oracle)
- RC-Oracle(FaceAppr) (variation oracle)
- RC-Oracle(Vertex) (variation oracle)
- RC-Oracle(Point) (our oracle)
- Fly(FaceExact) (on-the-fly baseline)
- Fly(FaceAppr) (on-the-fly baseline)
- Fly(Vertex) (variation on-the-fly)
- Fly(Point) (our on-the-fly)

Make sure there is a folder called "input/" and a folder called "output/" under the working directory. They will be used for storing the input/output files.

The source code are stored in "src/" folder.

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
- "GF_50_poi_on_10092.txt" (POI list with POI number of 50 on "GF_10092.xyz")
- "GF_100_poi_on_10092.txt" (POI list with POI number of 100 on "GF_10092.xyz")
- "GF_150_poi_on_10092.txt" (POI list with POI number of 150 on "GF_10092.xyz")
- "GF_200_poi_on_10092.txt" (POI list with POI number of 200 on "GF_10092.xyz")
- "GF_250_poi_on_10092.txt" (POI list with POI number of 250 on "GF_10092.xyz")
- "GF_50_poi_on_500208.txt" (POI list with POI number of 50 on "GF_500208.xyz")
- "GF_50_poi_on_1000518.txt" (POI list with POI number of 50 on "GF_1000518.xyz")
- "GF_50_poi_on_1501668.txt" (POI list with POI number of 50 on "GF_1501668.xyz")
- "GF_50_poi_on_2000832.txt" (POI list with POI number of 50 on "GF_2000832.xyz")
- "GF_50_poi_on_2502075.txt" (POI list with POI number of 50 on "GF_2502075.xyz")
- "GF_100_poi_on_500208.txt" (POI list with POI number of 100 on "GF_500208.xyz")
- "GF_150_poi_on_500208.txt" (POI list with POI number of 150 on "GF_500208.xyz")
- "GF_200_poi_on_500208.txt" (POI list with POI number of 200 on "GF_500208.xyz")
- "GF_250_poi_on_500208.txt" (POI list with POI number of 250 on "GF_500208.xyz")
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
- "LM_50_poi_on_500208.txt" (POI list with POI number of 50 on "LM_500208.xyz")
- "LM_50_poi_on_1000518.txt" (POI list with POI number of 50 on "LM_1000518.xyz")
- "LM_50_poi_on_1501668.txt" (POI list with POI number of 50 on "LM_1501668.xyz")
- "LM_50_poi_on_2000832.txt" (POI list with POI number of 50 on "LM_2000832.xyz")
- "LM_50_poi_on_2502075.txt" (POI list with POI number of 50 on "LM_2502075.xyz")
- "LM_100_poi_on_500208.txt" (POI list with POI number of 100 on "LM_500208.xyz")
- "LM_150_poi_on_500208.txt" (POI list with POI number of 150 on "LM_500208.xyz")
- "LM_200_poi_on_500208.txt" (POI list with POI number of 200 on "LM_500208.xyz")
- "LM_250_poi_on_500208.txt" (POI list with POI number of 250 on "LM_500208.xyz")
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
| 30 | BH | 500835 | 50 |
| 31 | BH | 500835 | 100 |
| 32 | BH | 500835 | 150 |
| 33 | BH | 500835 | 200 |
| 34 | BH | 500835 | 250 |
| 35 | BH | 1000414 | 50 |
| 36 | BH | 1500996 | 50 |
| 37 | BH | 2001610 | 50 |
| 38 | BH | 2502596 | 50 |
| 39 | EP | 500384 | 50 |
| 40 | EP | 500384 | 100 |
| 41 | EP | 500384 | 150 |
| 42 | EP | 500384 | 200 |
| 43 | EP | 500384 | 250 |
| 44 | EP | 1001040 | 50 |
| 45 | EP | 1501578 | 50 |
| 46 | EP | 2001536 | 50 |
| 47 | EP | 2500560 | 50 |
| 48 | GF | 500208 | 50 |
| 49 | GF | 500208 | 100 |
| 50 | GF | 500208 | 150 |
| 51 | GF | 500208 | 200 |
| 52 | GF | 500208 | 250 |
| 53 | GF | 1000518 | 50 |
| 54 | GF | 1501668 | 50 |
| 55 | GF | 2000832 | 50 |
| 56 | GF | 2502075 | 50 |
| 57 | LM | 500208 | 50 |
| 58 | LM | 500208 | 100 |
| 59 | LM | 500208 | 150 |
| 60 | LM | 500208 | 200 |
| 61 | LM | 500208 | 250 |
| 62 | LM | 1000518 | 50 |
| 63 | LM | 1501668 | 50 |
| 64 | LM | 2000832 | 50 |
| 65 | LM | 2502075 | 50 |
| 66 | RM | 500208 | 50 |
| 67 | RM | 500208 | 100 |
| 68 | RM | 500208 | 150 |
| 69 | RM | 500208 | 200 |
| 70 | RM | 500208 | 250 |
| 71 | RM | 1000518 | 50 |
| 72 | RM | 1501668 | 50 |
| 73 | RM | 2000832 | 50 |
| 74 | RM | 2502075 | 50 |
| 75 | BH | 500835 | 500 |
| 76 | BH | 500835 | 1000 |
| 77 | BH | 500835 | 1500 |
| 78 | BH | 500835 | 2000 |
| 79 | BH | 500835 | 2500 |
| 80 | BH | 1000414 | 500 |
| 81 | BH | 1500996 | 500 |
| 82 | BH | 2001610 | 500 |
| 83 | BH | 2502596 | 500 |
| 84 | EP | 500384 | 500 |
| 85 | EP | 500384 | 1000 |
| 86 | EP | 500384 | 1500 |
| 87 | EP | 500384 | 2000 |
| 88 | EP | 500384 | 2500 |
| 89 | EP | 1001040 | 500 |
| 90 | EP | 1501578 | 500 |
| 91 | EP | 2001536 | 500 |
| 92 | EP | 2500560 | 500 |
| 93 | GF | 500208 | 500 |
| 94 | GF | 500208 | 1000 |
| 95 | GF | 500208 | 1500 |
| 96 | GF | 500208 | 2000 |
| 97 | GF | 500208 | 2500 |
| 98 | GF | 1000518 | 500 |
| 99 | GF | 1501668 | 500 |
| 100 | GF | 2000832 | 500 |
| 101 | GF | 2502075 | 500 |
| 102 | LM | 500208 | 500 |
| 103 | LM | 500208 | 1000 |
| 104 | LM | 500208 | 1500 |
| 105 | LM | 500208 | 2000 |
| 106 | LM | 500208 | 2500 |
| 107 | LM | 1000518 | 500 |
| 108 | LM | 1501668 | 500 |
| 109 | LM | 2000832 | 500 |
| 110 | LM | 2502075 | 500 |
| 111 | RM | 500208 | 500 |
| 112 | RM | 500208 | 1000 |
| 113 | RM | 500208 | 1500 |
| 114 | RM | 500208 | 2000 |
| 115 | RM | 500208 | 2500 |
| 116 | RM | 1000518 | 500 |
| 117 | RM | 1501668 | 500 |
| 118 | RM | 2000832 | 500 |
| 119 | RM | 2502075 | 500 |

As mentioned in our paper, SE-Oracle(FaceExact), SE-Oracle(FaceAppr), SE-Oracle-Adapt(FaceExact), SE-Oracle-Adapt(FaceAppr), RC-Oracle-Naive(FaceExact), RC-Oracle-Naive(FaceAppr) are very time consuming (they are only feasible on small-version dataset), and SE-Oracle(Vertex), SE-Oracle(Point), SE-Oracle-Adapt(Vertex), SE-Oracle-Adapt(Point), RC-Oracle-Naive(Vertex), RC-Oracle-Naive(Point) are also time consuming (they are only feasible on small-verion dataset and large-version dataset with small-version POIs). So the project will run SE-Oracle(FaceExact), SE-Oracle(FaceAppr), SE-Oracle(Vertex), SE-Oracle(Point), SE-Oracle-Adapt(FaceExact), SE-Oracle-Adapt(FaceAppr), SE-Oracle-Adapt(Vertex), SE-Oracle-Adapt(Point), RC-Oracle-Naive(FaceExact), RC-Oracle-Naive(FaceAppr), RC-Oracle-Naive(Vertex), RC-Oracle-Naive(Point), RC-Oracle(FaceExact), RC-Oracle(FaceAppr), RC-Oracle(Vertex), RC-Oracle(Point), Fly(FaceExact), Fly(FaceAppr), Fly(Vertex) and Fly(Point) on small-version dataset ([point_cloud_data_and_point_number_and_poi_number_map_index] <= 29). The project will run SE-Oracle(Vertex), SE-Oracle(Point), SE-Oracle-Adapt(Vertex), SE-Oracle-Adapt(Point), RC-Oracle-Naive(Vertex), RC-Oracle-Naive(Point), RC-Oracle(FaceExact), RC-Oracle(FaceAppr), RC-Oracle(Vertex), RC-Oracle(Point), Fly(FaceExact), Fly(FaceAppr), Fly(Vertex) and Fly(Point) on large-version dataset with small-version POIs (29 < [point_cloud_data_and_point_number_and_poi_number_map_index] <= 74). The project will run RC-Oracle(FaceExact), RC-Oracle(FaceAppr), RC-Oracle(Vertex), RC-Oracle(Point), Fly(FaceExact), Fly(FaceAppr), Fly(Vertex) and Fly(Point) on large-version dataset with large-version POIs ([point_cloud_data_and_point_number_and_poi_number_map_index] > 74).

In addition, we strongly encourage you to set [run_knn] to 0 if you are not conducting experiments. Otherwise, it will take a very long time to run calculate the knn of all POIs. 

An example:

```
./main 0 0.5 0 0
```

In this example, [point_cloud_data_and_point_number_and_poi_number_map_index] is 0, [epsilon] is 0.5, [run_knn_query] is 0, [run_range_query] is 0. So, it will run BH point cloud dataset, with point number equal to 10086 and poi number equal to 50, epsilon is 0.5, it will not run all POIs knn query and will not run all POIs range query. It will run 20 algorithms.

## Output

The output will be stored in "output/output.txt" file. The format will be as follows:

```

[dataset] [point_num] [poi_num] [epsilon] [point_cloud_to_terrain_time (ms)] [construction_time (ms)] [query_time (ms)] [point_cloud_to_terrain_memroy_usage (MB)] [memory_usage (MB)] [index_size (MB)] [distance_error_point_cloud] [distance_error_terrain] [knn_query_time] [knn_error_point_cloud] [knn_error_terrain] [range_query_time] [range_error_point_cloud] [range_error_terrain]

```

These information will also be shown in the terminal. 