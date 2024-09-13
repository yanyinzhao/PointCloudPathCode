#include "algorithms.h"

struct input_struct
{
       std::string point_cloud_or_terrain;
       std::string poi;
       int source_poi_index;
       int destination_poi_index;

       input_struct() {}

       input_struct(std::string _point_cloud_or_terrain, std::string _poi,
                    int _source_poi_index, int _destination_poi_index)
       {
              point_cloud_or_terrain = _point_cloud_or_terrain;
              poi = _poi;
              source_poi_index = _source_poi_index;
              destination_poi_index = _destination_poi_index;
       }
};

int main(int argc, char **argv)
{
       int input_file_index = std::stoi(argv[1]);
       double e = std::stod(argv[2]);
       int run_knn_query_index = std::stod(argv[3]);
       int run_range_query_index = std::stod(argv[4]);

       std::string input_folder = "../input/";
       std::string input_poi_folder = "../input/";

       std::vector<input_struct> input_file;
       input_file.push_back(input_struct("BH_1014", "BH_50_poi_on_1014.txt", 0, 1));
       input_file.push_back(input_struct("BH_10086", "BH_50_poi_on_10086.txt", 0, 1));
       input_file.push_back(input_struct("BH_10086", "BH_100_poi_on_10086.txt", 0, 1));
       input_file.push_back(input_struct("BH_10086", "BH_150_poi_on_10086.txt", 0, 1));
       input_file.push_back(input_struct("BH_10086", "BH_200_poi_on_10086.txt", 0, 1));
       input_file.push_back(input_struct("BH_10086", "BH_250_poi_on_10086.txt", 0, 1));
       input_file.push_back(input_struct("EP_10062", "EP_50_poi_on_10062.txt", 0, 1));
       input_file.push_back(input_struct("EP_10062", "EP_100_poi_on_10062.txt", 0, 1));
       input_file.push_back(input_struct("EP_10062", "EP_150_poi_on_10062.txt", 0, 1));
       input_file.push_back(input_struct("EP_10062", "EP_200_poi_on_10062.txt", 0, 1));
       input_file.push_back(input_struct("EP_10062", "EP_250_poi_on_10062.txt", 0, 1));
       input_file.push_back(input_struct("EP_20130", "EP_50_poi_on_20130.txt", 0, 1));
       input_file.push_back(input_struct("EP_30098", "EP_50_poi_on_30098.txt", 0, 1));
       input_file.push_back(input_struct("EP_40076", "EP_50_poi_on_40076.txt", 0, 1));
       input_file.push_back(input_struct("EP_50373", "EP_50_poi_on_50373.txt", 0, 1));
       input_file.push_back(input_struct("GF_10092", "GF_50_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("GF_10092", "GF_100_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("GF_10092", "GF_150_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("GF_10092", "GF_200_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("GF_10092", "GF_250_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("LM_10092", "LM_50_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("LM_10092", "LM_100_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("LM_10092", "LM_150_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("LM_10092", "LM_200_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("LM_10092", "LM_250_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("RM_10092", "RM_50_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("RM_10092", "RM_100_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("RM_10092", "RM_150_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("RM_10092", "RM_200_poi_on_10092.txt", 0, 1));
       input_file.push_back(input_struct("RM_10092", "RM_250_poi_on_10092.txt", 0, 1));

       input_file.push_back(input_struct("BH_500835", "BH_500_poi_on_500835.txt", 0, 1));
       input_file.push_back(input_struct("BH_500835", "BH_1000_poi_on_500835.txt", 0, 1));
       input_file.push_back(input_struct("BH_500835", "BH_1500_poi_on_500835.txt", 0, 1));
       input_file.push_back(input_struct("BH_500835", "BH_2000_poi_on_500835.txt", 0, 1));
       input_file.push_back(input_struct("BH_500835", "BH_2500_poi_on_500835.txt", 0, 1));
       input_file.push_back(input_struct("BH_1000414", "BH_500_poi_on_1000414.txt", 0, 1));
       input_file.push_back(input_struct("BH_1500996", "BH_500_poi_on_1500996.txt", 0, 1));
       input_file.push_back(input_struct("BH_2001610", "BH_500_poi_on_2001610.txt", 0, 1));
       input_file.push_back(input_struct("BH_2502596", "BH_500_poi_on_2502596.txt", 0, 1));
       input_file.push_back(input_struct("EP_500384", "EP_500_poi_on_500384.txt", 0, 1));
       input_file.push_back(input_struct("EP_500384", "EP_1000_poi_on_500384.txt", 0, 1));
       input_file.push_back(input_struct("EP_500384", "EP_1500_poi_on_500384.txt", 0, 1));
       input_file.push_back(input_struct("EP_500384", "EP_2000_poi_on_500384.txt", 0, 1));
       input_file.push_back(input_struct("EP_500384", "EP_2500_poi_on_500384.txt", 0, 1));
       input_file.push_back(input_struct("EP_1001040", "EP_500_poi_on_1001040.txt", 0, 1));
       input_file.push_back(input_struct("EP_1501578", "EP_500_poi_on_1501578.txt", 0, 1));
       input_file.push_back(input_struct("EP_2001536", "EP_500_poi_on_2001536.txt", 0, 1));
       input_file.push_back(input_struct("EP_2500560", "EP_500_poi_on_2500560.txt", 0, 1));
       input_file.push_back(input_struct("GF_500208", "GF_500_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("GF_500208", "GF_1000_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("GF_500208", "GF_1500_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("GF_500208", "GF_2000_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("GF_500208", "GF_2500_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("GF_1000518", "GF_500_poi_on_1000518.txt", 0, 1));
       input_file.push_back(input_struct("GF_1501668", "GF_500_poi_on_1501668.txt", 0, 1));
       input_file.push_back(input_struct("GF_2000832", "GF_500_poi_on_2000832.txt", 0, 1));
       input_file.push_back(input_struct("GF_2502075", "GF_500_poi_on_2502075.txt", 0, 1));
       input_file.push_back(input_struct("LM_500208", "LM_500_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("LM_500208", "LM_1000_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("LM_500208", "LM_1500_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("LM_500208", "LM_2000_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("LM_500208", "LM_2500_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("LM_1000518", "LM_500_poi_on_1000518.txt", 0, 1));
       input_file.push_back(input_struct("LM_1501668", "LM_500_poi_on_1501668.txt", 0, 1));
       input_file.push_back(input_struct("LM_2000832", "LM_500_poi_on_2000832.txt", 0, 1));
       input_file.push_back(input_struct("LM_2502075", "LM_500_poi_on_2502075.txt", 0, 1));
       input_file.push_back(input_struct("RM_500208", "RM_500_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("RM_500208", "RM_1000_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("RM_500208", "RM_1500_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("RM_500208", "RM_2000_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("RM_500208", "RM_2500_poi_on_500208.txt", 0, 1));
       input_file.push_back(input_struct("RM_1000518", "RM_500_poi_on_1000518.txt", 0, 1));
       input_file.push_back(input_struct("RM_1501668", "RM_500_poi_on_1501668.txt", 0, 1));
       input_file.push_back(input_struct("RM_2000832", "RM_500_poi_on_2000832.txt", 0, 1));
       input_file.push_back(input_struct("RM_2502075", "RM_500_poi_on_2502075.txt", 0, 1));

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

       std::string input_point_cloud = input_folder + input_file[input_file_index].point_cloud_or_terrain + ".xyz";
       std::string input_terrain = input_folder + input_file[input_file_index].point_cloud_or_terrain + ".off";
       std::string input_poi = input_poi_folder + input_file[input_file_index].poi;
       int source_poi_index = input_file[input_file_index].source_poi_index;
       int destination_poi_index = input_file[input_file_index].destination_poi_index;
       assert(source_poi_index != destination_poi_index);

       // std::cout.precision(10);

       std::vector<double> points;
       point_cloud_geodesic::read_point_cloud_from_file(&input_point_cloud[0], points);
       point_cloud_geodesic::PointCloud point_cloud;
       point_cloud.initialize_point_cloud_data(points);

       std::vector<double> terrain_points;
       std::vector<unsigned> terrain_faces;
       geodesic::read_mesh_from_file(&input_terrain[0], terrain_points, terrain_faces);
       geodesic::Mesh mesh;
       mesh.initialize_mesh_data(terrain_points, terrain_faces);

       std::vector<int> poi_list;
       int poi_num;
       std::ifstream input(&input_poi[0], std::ios::in);
       input >> poi_num;
       poi_list.resize(poi_num);
       for (int i = 0; i < poi_num; i++)
       {
              input >> poi_list[i];
       }

       std::string write_file_header = input_file[input_file_index].point_cloud_or_terrain + "\t" +
                                       std::to_string(point_cloud.pc_points().size()) + "\t" +
                                       std::to_string(poi_num) + "\t" +
                                       std::to_string(e);

       std::cout << "dataset: " << input_file[input_file_index].point_cloud_or_terrain << "\tdataset_size: " << point_cloud.pc_points().size() << "\tpoi_num: " << poi_num << "\tepsilon: " << e << std::endl;
       std::cout << std::endl;

       std::string output_file = "../output/output.txt";
       std::ofstream ofs(output_file, std::ofstream::app);
       ofs << "# dataset\tdataset_size\tpoi_num\tepsilon\tpoint_cloud_to_terrain_time\tconstruction_time\tquery_time\tpoint_cloud_to_terrain_memroy_usage\tmemory_usage\toutput_size\tdistance_error_point_cloud\tdistance_error_terrain\tknn_query_time\tknn_error_point_cloud\tknn_error_terrain\trange_query_time\trange_error_point_cloud\trange_error_terrain\n\n";
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
       calculate_terrain_exact_distance(&mesh, poi_list, source_poi_index, destination_poi_index, terrain_exact_distance);
       if (run_knn_query)
       {
              calculate_point_cloud_exact_all_poi_knn_or_range_query(&point_cloud, poi_list, 1, k_value, range, point_cloud_exact_all_poi_knn_query_list);
              calculate_terrain_exact_all_poi_knn_or_range_query(&mesh, poi_list, 1, k_value, range, terrain_exact_all_poi_knn_query_list);
       }
       if (run_range_query)
       {
              calculate_point_cloud_exact_all_poi_knn_or_range_query(&point_cloud, poi_list, 2, k_value, range, point_cloud_exact_all_poi_range_query_list);
              calculate_terrain_exact_all_poi_knn_or_range_query(&mesh, poi_list, 2, k_value, range, terrain_exact_all_poi_range_query_list);
       }

       if (input_file_index >= 0 && input_file_index <= 29)
       {
              std::cout << "== SE_Oracle_Adapt ==" << std::endl;
              SE_Oracle_or_SE_Oracle_Adapt_with_output(output_file, poi_num, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                                       terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                                       terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                                       write_file_header, 1);
              std::cout << std::endl;

              std::cout << "== EAR_Oracle_Adapt ==" << std::endl;
              EAR_Oracle_or_EAR_Oracle_Adapt_with_output(output_file, poi_num, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                                         terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                                         terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                                         write_file_header, 1);
              std::cout << std::endl;

              std::cout << "== SU_Oracle_Adapt ==" << std::endl;
              SU_Oracle_Adapt_with_output(output_file, poi_num, &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                          terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                          terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                          write_file_header);
              std::cout << std::endl;

              std::cout << "== RC_Oracle_Naive ==" << std::endl;
              RC_Oracle_Naive_or_RC_Oracle_Naive_Adapt_with_output(output_file, poi_num, &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                                                   terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                                                   terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                                                   write_file_header, 1);
              std::cout << std::endl;

              std::cout << "== SE_Oracle_Adapt_A2A ==" << std::endl;
              SE_Oracle_Adapt_A2A_with_output(output_file, poi_num, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                              terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                              terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                              write_file_header);
              std::cout << std::endl;

              std::cout << "== RC_Oracle_Naive_A2A ==" << std::endl;
              RC_Oracle_Naive_A2A_with_output(output_file, poi_num, &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                              terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                              terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                              write_file_header);
              std::cout << std::endl;

              std::cout << "== SE_Oracle ==" << std::endl;
              SE_Oracle_or_SE_Oracle_Adapt_with_output(output_file, poi_num, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                                       terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                                       terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                                       write_file_header, 2);
              std::cout << std::endl;

              std::cout << "== EAR_Oracle ==" << std::endl;
              EAR_Oracle_or_EAR_Oracle_Adapt_with_output(output_file, poi_num, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                                         terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                                         terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                                         write_file_header, 2);
              std::cout << std::endl;

              std::cout << "== RC_Oracle_Naive_Adapt ==" << std::endl;
              RC_Oracle_Naive_or_RC_Oracle_Naive_Adapt_with_output(output_file, poi_num, &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                                                   terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                                                   terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                                                   write_file_header, 2);
              std::cout << std::endl;
       }

       std::cout << "== RC_Oracle ==" << std::endl;
       RC_Oracle_or_RC_Oracle_Adapt_with_output(output_file, poi_num, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                                terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                                terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                                write_file_header, 1);
       std::cout << std::endl;

       std::cout << "== SE_Oracle_FastFly_Adapt ==" << std::endl;
       SE_Oracle_FastFly_Adapt_with_output(output_file, poi_num, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                           terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                           terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                           write_file_header);
       std::cout << std::endl;

       std::cout << "== EAR_Oracle_FastFly_Adapt ==" << std::endl;
       EAR_Oracle_FastFly_Adapt_with_output(output_file, poi_num, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                            terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                            terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                            write_file_header);
       std::cout << std::endl;

       std::cout << "== RC_Oracle_A2A ==" << std::endl;
       RC_Oracle_A2A_with_output(output_file, poi_num, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                 terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                 terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                 write_file_header);
       std::cout << std::endl;

       std::cout << "== SE_Oracle_FastFly_Adapt_A2A ==" << std::endl;
       SE_Oracle_FastFly_Adapt_A2A_with_output(output_file, poi_num, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                               terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                               terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                               write_file_header);
       std::cout << std::endl;

       std::cout << "== RC_Oracle_NaiveProx ==" << std::endl;
       RC_Oracle_NaiveProx_with_output(output_file, poi_num, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                       terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                       terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                       write_file_header);
       std::cout << std::endl;

       std::cout << "== RC_Oracle_NaiveProx_A2A ==" << std::endl;
       RC_Oracle_NaiveProx_A2A_with_output(output_file, poi_num, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                           terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                           terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                           write_file_header);
       std::cout << std::endl;

       std::cout << "== CH_Adapt ==" << std::endl;
       CH_or_CH_Adapt_with_output(output_file, &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                  terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                  terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                  write_file_header, 1);
       std::cout << std::endl;

       std::cout << "== Kaul_Adapt ==" << std::endl;
       Kaul_or_Kaul_Adapt_with_output(output_file, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                      terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                      terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                      write_file_header, 1);
       std::cout << std::endl;

       std::cout << "== Dijk_Adapt ==" << std::endl;
       Dijk_or_Dijk_Adapt_with_output(output_file, &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                      terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                      terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                      write_file_header, 1);
       std::cout << std::endl;

       std::cout << "== FastFly ==" << std::endl;
       FastFly_or_FastFly_Adapt_with_output(output_file, &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                            terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                            terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                            write_file_header, 1);
       std::cout << std::endl;

       std::cout << "== RC_Oracle_Adapt ==" << std::endl;
       RC_Oracle_or_RC_Oracle_Adapt_with_output(output_file, poi_num, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                                terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                                terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                                write_file_header, 2);
       std::cout << std::endl;

       std::cout << "== CH ==" << std::endl;
       CH_or_CH_Adapt_with_output(output_file, &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                  terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                  terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                  write_file_header, 2);
       std::cout << std::endl;

       std::cout << "== Kaul ==" << std::endl;
       Kaul_or_Kaul_Adapt_with_output(output_file, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                      terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                      terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                      write_file_header, 2);
       std::cout << std::endl;

       std::cout << "== Dijk ==" << std::endl;
       Dijk_or_Dijk_Adapt_with_output(output_file, &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                      terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                      terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                      write_file_header, 2);
       std::cout << std::endl;

       std::cout << "== FastFly_Adapt ==" << std::endl;
       FastFly_or_FastFly_Adapt_with_output(output_file, &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                            terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                            terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                            write_file_header, 2);
       std::cout << std::endl;

       input.close();
}
