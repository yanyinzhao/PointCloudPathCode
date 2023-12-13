#include "algorithms.h"

struct input_struct
{
       std::string point_cloud;
       std::string poi;
       int source_poi_index;
       int destination_poi_index;

       input_struct() {}

       input_struct(std::string _point_cloud, std::string _poi,
                    int _source_poi_index, int _destination_poi_index)
       {
              point_cloud = _point_cloud;
              poi = _poi;
              source_poi_index = _source_poi_index;
              destination_poi_index = _destination_poi_index;
       }
};

int main(int argc, char **argv)
{
       int input_file_index = std::stoi(argv[1]);
       double epsilon = std::stod(argv[2]);
       int run_knn_query_index = std::stod(argv[3]);
       int run_range_query_index = std::stod(argv[4]);

       std::string input_folder = "../input/";
       std::string input_poi_folder = "../input/";

       std::vector<input_struct> input_file;
       input_file.push_back(input_struct("BH_1014.xyz", "BH_50_poi_on_1014.txt", 0, 1));
       input_file.push_back(input_struct("BH_10086.xyz", "BH_50_poi_on_10086.txt", 0, 1));
       input_file.push_back(input_struct("BH_10086.xyz", "BH_100_poi_on_10086.txt", 0, 1));
       input_file.push_back(input_struct("BH_10086.xyz", "BH_150_poi_on_10086.txt", 0, 1));
       input_file.push_back(input_struct("BH_10086.xyz", "BH_200_poi_on_10086.txt", 0, 1));
       input_file.push_back(input_struct("BH_10086.xyz", "BH_250_poi_on_10086.txt", 0, 1));
       input_file.push_back(input_struct("EP_10062.xyz", "EP_50_poi_on_10062.txt", 0, 1));
       input_file.push_back(input_struct("EP_10062.xyz", "EP_100_poi_on_10062.txt", 0, 1));
       input_file.push_back(input_struct("EP_10062.xyz", "EP_150_poi_on_10062.txt", 0, 1));
       input_file.push_back(input_struct("EP_10062.xyz", "EP_200_poi_on_10062.txt", 0, 1));
       input_file.push_back(input_struct("EP_10062.xyz", "EP_250_poi_on_10062.txt", 0, 1));
       input_file.push_back(input_struct("EP_20130.xyz", "EP_50_poi_on_20130.txt", 0, 1));
       input_file.push_back(input_struct("EP_30098.xyz", "EP_50_poi_on_30098.txt", 0, 1));
       input_file.push_back(input_struct("EP_40076.xyz", "EP_50_poi_on_40076.txt", 0, 1));
       input_file.push_back(input_struct("EP_50373.xyz", "EP_50_poi_on_50373.txt", 0, 1));
       input_file.push_back(input_struct("GF_10092.xyz", "GF_50_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("GF_10092.xyz", "GF_100_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("GF_10092.xyz", "GF_150_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("GF_10092.xyz", "GF_200_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("GF_10092.xyz", "GF_250_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("LM_10092.xyz", "LM_50_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("LM_10092.xyz", "LM_100_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("LM_10092.xyz", "LM_150_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("LM_10092.xyz", "LM_200_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("LM_10092.xyz", "LM_250_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("RM_10092.xyz", "RM_50_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("RM_10092.xyz", "RM_100_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("RM_10092.xyz", "RM_150_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("RM_10092.xyz", "RM_200_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("RM_10092.xyz", "RM_250_poi_on_10092.txt", 0, 1));

       input_file.push_back(input_struct("BH_500835.xyz", "BH_500_poi_on_500835.txt", 0, 1));
       input_file.push_back(input_struct("BH_500835.xyz", "BH_1000_poi_on_500835.txt", 0, 1));
       input_file.push_back(input_struct("BH_500835.xyz", "BH_1500_poi_on_500835.txt", 0, 1));
       input_file.push_back(input_struct("BH_500835.xyz", "BH_2000_poi_on_500835.txt", 0, 1));
       input_file.push_back(input_struct("BH_500835.xyz", "BH_2500_poi_on_500835.txt", 0, 1));
       input_file.push_back(input_struct("BH_1000414.xyz", "BH_500_poi_on_1000414.txt", 0, 1));
       input_file.push_back(input_struct("BH_1500996.xyz", "BH_500_poi_on_1500996.txt", 0, 1));
       input_file.push_back(input_struct("BH_2001610.xyz", "BH_500_poi_on_2001610.txt", 0, 1));
       input_file.push_back(input_struct("BH_2502596.xyz", "BH_500_poi_on_2502596.txt", 0, 1));
       input_file.push_back(input_struct("EP_500384.xyz", "EP_500_poi_on_500384.txt", 0, 1));
       input_file.push_back(input_struct("EP_500384.xyz", "EP_1000_poi_on_500384.txt", 0, 1));
       input_file.push_back(input_struct("EP_500384.xyz", "EP_1500_poi_on_500384.txt", 0, 1));
       input_file.push_back(input_struct("EP_500384.xyz", "EP_2000_poi_on_500384.txt", 0, 1));
       input_file.push_back(input_struct("EP_500384.xyz", "EP_2500_poi_on_500384.txt", 0, 1));
       input_file.push_back(input_struct("EP_1001040.xyz", "EP_500_poi_on_1001040.txt", 0, 1));
       input_file.push_back(input_struct("EP_1501578.xyz", "EP_500_poi_on_1501578.txt", 0, 1));
       input_file.push_back(input_struct("EP_2001536.xyz", "EP_500_poi_on_2001536.txt", 0, 1));
       input_file.push_back(input_struct("EP_2500560.xyz", "EP_500_poi_on_2500560.txt", 0, 1));
       input_file.push_back(input_struct("GF_500208.xyz", "GF_500_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("GF_500208.xyz", "GF_1000_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("GF_500208.xyz", "GF_1500_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("GF_500208.xyz", "GF_2000_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("GF_500208.xyz", "GF_2500_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("GF_1000518.xyz", "GF_500_poi_on_1000518.txt", 0, 1));
       input_file.push_back(input_struct("GF_1501668.xyz", "GF_500_poi_on_1501668.txt", 0, 1));
       input_file.push_back(input_struct("GF_2000832.xyz", "GF_500_poi_on_2000832.txt", 0, 1));
       input_file.push_back(input_struct("GF_2502075.xyz", "GF_500_poi_on_2502075.txt", 0, 1));
       input_file.push_back(input_struct("LM_500208.xyz", "LM_500_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("LM_500208.xyz", "LM_1000_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("LM_500208.xyz", "LM_1500_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("LM_500208.xyz", "LM_2000_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("LM_500208.xyz", "LM_2500_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("LM_1000518.xyz", "LM_500_poi_on_1000518.txt", 0, 1));
       input_file.push_back(input_struct("LM_1501668.xyz", "LM_500_poi_on_1501668.txt", 0, 1));
       input_file.push_back(input_struct("LM_2000832.xyz", "LM_500_poi_on_2000832.txt", 0, 1));
       input_file.push_back(input_struct("LM_2502075.xyz", "LM_500_poi_on_2502075.txt", 0, 1));
       input_file.push_back(input_struct("RM_500208.xyz", "RM_500_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("RM_500208.xyz", "RM_1000_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("RM_500208.xyz", "RM_1500_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("RM_500208.xyz", "RM_2000_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("RM_500208.xyz", "RM_2500_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("RM_1000518.xyz", "RM_500_poi_on_1000518.txt", 0, 1));
       input_file.push_back(input_struct("RM_1501668.xyz", "RM_500_poi_on_1501668.txt", 0, 1));
       input_file.push_back(input_struct("RM_2000832.xyz", "RM_500_poi_on_2000832.txt", 0, 1));
       input_file.push_back(input_struct("RM_2502075.xyz", "RM_500_poi_on_2502075.txt", 0, 1));

       int k_value = 3;
       double range = 100;
       bool run_knn_query = false;
       bool run_range_query = false;
       if (run_knn_query_index == 1)
       {
              run_knn_query = true;
       }
       if (run_range_query_index == 1)
       {
              run_range_query = true;
       }

       std::string input_point_cloud = input_folder + input_file[input_file_index].point_cloud;
       std::string input_poi = input_poi_folder + input_file[input_file_index].poi;
       int source_poi_index = input_file[input_file_index].source_poi_index;
       int destination_poi_index = input_file[input_file_index].destination_poi_index;
       assert(source_poi_index != destination_poi_index);

       // std::cout.precision(10);

       std::vector<double> points;
       point_cloud_geodesic::read_point_cloud_from_file(&input_point_cloud[0], points);
       point_cloud_geodesic::PointCloud point_cloud;
       point_cloud.initialize_point_cloud_data(points);

       std::vector<int> poi_list;
       int poi_num;
       std::ifstream input(&input_poi[0], std::ios::in);
       input >> poi_num;
       poi_list.resize(poi_num);
       for (int i = 0; i < poi_num; i++)
       {
              input >> poi_list[i];
       }

       std::string write_file_header = input_file[input_file_index].point_cloud + "\t" +
                                       std::to_string(point_cloud.pc_points().size()) + "\t" +
                                       std::to_string(poi_num) + "\t" +
                                       std::to_string(epsilon);

       std::cout << "dataset: " << input_file[input_file_index].point_cloud << "\tpoint_num: " << point_cloud.pc_points().size() << "\tpoi_num: " << poi_num << "\tepsilon: " << epsilon << std::endl;
       std::cout << std::endl;

       std::ofstream ofs("../output/output.txt", std::ofstream::app);
       ofs << "# dataset\tpoint_num\tpoi_num\tepsilon\tpoint_cloud_to_terrain_time\tconstruction_time\tquery_time\tpoint_cloud_to_terrain_memroy_usage\tmemory_usage\tindex_size\tdistance_error_point_cloud\tdistance_error_terrain\tknn_query_time\tknn_error_point_cloud\tknn_error_terrain\trange_query_time\trange_error_point_cloud\trange_error_terrain\n\n";
       ofs.close();

       assert(source_poi_index <= poi_num - 1 && destination_poi_index <= poi_num - 1);

       double point_cloud_exact_distance = 0;
       double terrain_exact_distance = 0;
       std::vector<std::vector<int>> point_cloud_exact_all_poi_knn_query_list;
       std::vector<std::vector<int>> terrain_exact_all_poi_knn_query_list;
       std::vector<std::vector<int>> point_cloud_exact_all_poi_range_query_list;
       std::vector<std::vector<int>> terrain_exact_all_poi_range_query_list;
       point_cloud_exact_all_poi_knn_query_list.clear();
       terrain_exact_all_poi_knn_query_list.clear();
       point_cloud_exact_all_poi_range_query_list.clear();
       terrain_exact_all_poi_range_query_list.clear();

       calculate_point_cloud_exact_distance(&point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance);
       calculate_terrain_exact_distance(&point_cloud, poi_list, source_poi_index, destination_poi_index, terrain_exact_distance);
       if (run_knn_query)
       {
              calculate_point_cloud_exact_all_poi_knn_or_range_query(&point_cloud, poi_list, 1, k_value, range, point_cloud_exact_all_poi_knn_query_list);
              calculate_terrain_exact_all_poi_knn_or_range_query(&point_cloud, poi_list, 1, k_value, range, terrain_exact_all_poi_knn_query_list);
       }
       if (run_range_query)
       {
              calculate_point_cloud_exact_all_poi_knn_or_range_query(&point_cloud, poi_list, 2, k_value, range, point_cloud_exact_all_poi_range_query_list);
              calculate_terrain_exact_all_poi_knn_or_range_query(&point_cloud, poi_list, 2, k_value, range, terrain_exact_all_poi_range_query_list);
       }

       if (input_file_index >= 0 && input_file_index <= 29)
       {
              std::cout << "== SE_Oracle_Adapt ==" << std::endl;
              SE_Oracle_Adapt_with_output(poi_num, &point_cloud, poi_list, epsilon, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                          terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                          terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                          write_file_header);
              std::cout << std::endl;

              std::cout << "== EAR_Oracle_Adapt ==" << std::endl;
              EAR_Oracle_Adapt_with_output(poi_num, &point_cloud, poi_list, epsilon, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                           terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                           terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                           write_file_header);
              std::cout << std::endl;

              std::cout << "== SI_Oracle_Adapt ==" << std::endl;
              SI_Oracle_Adapt_with_output(poi_num, &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                          terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                          terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                          write_file_header);
              std::cout << std::endl;

              std::cout << "== RC_Oracle_Naive ==" << std::endl;
              RC_Oracle_Naive_with_output(poi_num, &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                          terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                          terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                          write_file_header);
              std::cout << std::endl;
       }

       std::cout << "== RC_Oracle_NaiveProx ==" << std::endl;
       RC_Oracle_NaiveProx_with_output(poi_num, &point_cloud, poi_list, epsilon, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                       terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                       terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                       write_file_header);
       std::cout << std::endl;

       std::cout << "== RC_Oracle ==" << std::endl;
       RC_Oracle_with_output(poi_num, &point_cloud, poi_list, epsilon, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                             terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                             terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                             write_file_header);
       std::cout << std::endl;

       std::cout << "== SE_Oracle_FastFly_Adapt ==" << std::endl;
       SE_Oracle_FastFly_Adapt_with_output(poi_num, &point_cloud, poi_list, epsilon, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                           terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                           terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                           write_file_header);
       std::cout << std::endl;

       std::cout << "== EAR_Oracle_FastFly_Adapt ==" << std::endl;
       EAR_Oracle_FastFly_Adapt_with_output(poi_num, &point_cloud, poi_list, epsilon, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                            terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                            terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                            write_file_header);
       std::cout << std::endl;

       std::cout << "== CH_Adapt ==" << std::endl;
       CH_Adapt_with_output(&point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                            terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                            terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                            write_file_header);
       std::cout << std::endl;

       std::cout << "== Kaul_Adapt ==" << std::endl;
       Kaul_Adapt_with_output(&point_cloud, poi_list, epsilon, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                              terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                              terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                              write_file_header);
       std::cout << std::endl;

       std::cout << "== Dijk_Adapt ==" << std::endl;
       Dijk_Adapt_with_output(&point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                              terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                              terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                              write_file_header);
       std::cout << std::endl;

       std::cout << "== FastFly_Adapt ==" << std::endl;
       FastFly_with_output(&point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                           terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                           terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                           write_file_header);
       std::cout << std::endl;

       input.close();
}
