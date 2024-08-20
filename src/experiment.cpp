#include "algorithms.h"
#include <filesystem>

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

void run_algorithms(std::map<int, input_struct> input_file,
                    std::string output_folder,
                    double epsilon, int input_file_index, int algorithm_type)
{
    int k_value = 3;
    double range = 100;
    bool run_knn_query = false;
    bool run_range_query = false;

    std::string input_folder = "../input/";
    std::string input_poi_folder = "../input/";

    std::string input_point_cloud = input_folder + input_file[input_file_index].point_cloud;
    std::string input_poi = input_poi_folder + input_file[input_file_index].poi;
    int source_poi_index = input_file[input_file_index].source_poi_index;
    int destination_poi_index = input_file[input_file_index].destination_poi_index;
    assert(source_poi_index != destination_poi_index);

    std::cout << "============" << std::endl;

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

    if (algorithm_type == 1)
    {
        std::cout << "== SE_Oracle_Adapt ==" << std::endl;
        SE_Oracle_Adapt_with_output(output_folder + "1_SE_Oracle_Adapt.txt", poi_num, &point_cloud, poi_list, epsilon, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                    terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                    terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                    write_file_header);
        std::cout << std::endl;

        std::cout << "== EAR_Oracle_Adapt ==" << std::endl;
        EAR_Oracle_Adapt_with_output(output_folder + "2_EAR_Oracle_Adapt.txt", poi_num, &point_cloud, poi_list, epsilon, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                     terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                     terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                     write_file_header);
        std::cout << std::endl;

        std::cout << "== RC_Oracle_Naive ==" << std::endl;
        RC_Oracle_Naive_with_output(output_folder + "3_RC_Oracle_Naive.txt", poi_num, &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                    terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                    terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                    write_file_header);
        std::cout << std::endl;
    }

    std::cout << "== RC_Oracle ==" << std::endl;
    RC_Oracle_with_output(output_folder + "4_RC_Oracle.txt", poi_num, &point_cloud, poi_list, epsilon, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                          terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                          terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                          write_file_header);
    std::cout << std::endl;

    if (algorithm_type == 1 || algorithm_type == 2)
    {
        std::cout << "== CH_Adapt ==" << std::endl;
        CH_Adapt_with_output(output_folder + "5_CH_Adapt.txt", &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                             terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                             terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                             write_file_header);
        std::cout << std::endl;

        std::cout << "== Kaul_Adapt ==" << std::endl;
        Kaul_Adapt_with_output(output_folder + "6_Kaul_Adapt.txt", &point_cloud, poi_list, epsilon, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                               terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                               terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                               write_file_header);
        std::cout << std::endl;

        std::cout << "== Dijk_Adapt ==" << std::endl;
        Dijk_Adapt_with_output(output_folder + "7_Dijk_Adapt.txt", &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                               terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                               terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                               write_file_header);
        std::cout << std::endl;

        std::cout << "== FastFly ==" << std::endl;
        FastFly_with_output(output_folder + "8_FastFly.txt", &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                            terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                            terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                            write_file_header);
        std::cout << std::endl;
    }

    if (algorithm_type == 3)
    {
        std::cout << "== SE_Oracle_FastFly_Adapt ==" << std::endl;
        SE_Oracle_FastFly_Adapt_with_output(
            output_folder + "9_SE_Oracle_FastFly_Adapt.txt", poi_num, &point_cloud, poi_list, epsilon, source_poi_index, destination_poi_index, point_cloud_exact_distance,
            terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
            terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
            write_file_header);
        std::cout << std::endl;

        std::cout << "== EAR_Oracle_FastFly_Adapt ==" << std::endl;
        EAR_Oracle_FastFly_Adapt_with_output(
            output_folder + "10_EAR_Oracle_FastFly_Adapt.txt", poi_num, &point_cloud, poi_list, epsilon, source_poi_index, destination_poi_index, point_cloud_exact_distance,
            terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
            terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
            write_file_header);
        std::cout << std::endl;
    }

    if (algorithm_type == 4)
    {
        std::cout << "== SU_Oracle_Adapt ==" << std::endl;
        SU_Oracle_Adapt_with_output(output_folder + "11_SU_Oracle_Adapt.txt", poi_num, &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                    terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                    terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                    write_file_header);
        std::cout << std::endl;

        std::cout << "== RC_Oracle_NaiveProx ==" << std::endl;
        RC_Oracle_NaiveProx_with_output(output_folder + "12_RC_Oracle_NaiveProx.txt", poi_num, &point_cloud, poi_list, epsilon, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                        terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                        terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                        write_file_header);
        std::cout << std::endl;
    }

    input.close();
}

int main(int argc, char **argv)
{
    // std::__fs::filesystem::remove_all("../exp");

    std::vector<std::string> output_folder_list = {
        "../exp/1_e_BH_small/",
        "../exp/2_n_BH_small/",
        "../exp/3_e_EP_small/",
        "../exp/4_n_EP_small/",
        "../exp/5_DS_EP_small/",
        "../exp/6_e_GF_small/",
        "../exp/7_n_GF_small/",
        "../exp/8_e_LM_small/",
        "../exp/9_n_LM_small/",
        "../exp/10_e_RM_small/",
        "../exp/11_n_RM_small/",
        "../exp/12_e_BH/",
        "../exp/13_n_BH/",
        "../exp/14_DS_BH/",
        "../exp/15_e_EP/",
        "../exp/16_n_EP/",
        "../exp/17_DS_EP/",
        "../exp/18_e_GF/",
        "../exp/19_n_GF/",
        "../exp/20_DS_GF/",
        "../exp/21_e_LM/",
        "../exp/22_n_LM/",
        "../exp/23_DS_LM/",
        "../exp/24_e_RM/",
        "../exp/25_n_RM/",
        "../exp/26_DS_RM/",
        "../exp/27_e_BH_abla/",
        "../exp/28_e_EP_abla/",
        "../exp/29_e_GF_abla/",
        "../exp/30_e_LM_abla/",
        "../exp/31_e_RM_abla/",
        "../exp/32_e_BH_small_proxim_ora/",
        "../exp/33_e_EP_small_proxim_ora/",
        "../exp/34_e_GF_small_proxim_ora/",
        "../exp/35_e_LM_small_proxim_ora/",
        "../exp/36_e_RM_small_proxim_ora/"};
    std::vector<std::string> output_file_list = {"1_SE_Oracle_Adapt.txt",
                                                 "2_EAR_Oracle_Adapt.txt",
                                                 "3_RC_Oracle_Naive.txt",
                                                 "4_RC_Oracle.txt",
                                                 "5_CH_Adapt.txt",
                                                 "6_Kaul_Adapt.txt",
                                                 "7_Dijk_Adapt.txt",
                                                 "8_FastFly.txt",
                                                 "9_SE_Oracle_FastFly_Adapt.txt",
                                                 "10_EAR_Oracle_FastFly_Adapt.txt",
                                                 "11_SU_Oracle_Adapt.txt",
                                                 "12_RC_Oracle_NaiveProx.txt"};
    for (int i = 0; i < output_folder_list.size(); i++)
    {
        for (int j = 0; j < output_file_list.size(); j++)
        {
            std::__fs::filesystem::remove(output_folder_list[i] + output_file_list[j]);
        }
    }

    for (int i = 0; i < output_folder_list.size(); i++)
    {
        std::__fs::filesystem::create_directories(output_folder_list[i]);
    }

    std::vector<double> epsilon_list = {0.05, 0.1, 0.25, 0.5, 0.75, 1};

    // run BH small dataset on small POI when varying epsilon
    std::map<int, input_struct> input_file1{
        {0, input_struct("BH_10086.xyz", "BH_50_poi_on_10086.txt", 0, 1)},
    };
    for (int i = 0; i < input_file1.size(); i++)
    {
        for (int j = 0; j < epsilon_list.size(); j++)
        {
            double epsilon = epsilon_list[j];
            int input_file_index = i;

            run_algorithms(input_file1, output_folder_list[0], epsilon, input_file_index, 1);
        }
    }

    // run BH small dataset on small POI when varying POI number
    std::map<int, input_struct> input_file2{
        {0, input_struct("BH_10086.xyz", "BH_50_poi_on_10086.txt", 0, 1)},
        {1, input_struct("BH_10086.xyz", "BH_100_poi_on_10086.txt", 0, 1)},
        {2, input_struct("BH_10086.xyz", "BH_150_poi_on_10086.txt", 0, 1)},
        {3, input_struct("BH_10086.xyz", "BH_200_poi_on_10086.txt", 0, 1)},
        {4, input_struct("BH_10086.xyz", "BH_250_poi_on_10086.txt", 0, 1)},
    };
    for (int i = 0; i < input_file2.size(); i++)
    {
        double epsilon = 0.1;
        int input_file_index = i;

        run_algorithms(input_file2, output_folder_list[1], epsilon, input_file_index, 1);
    }

    // run EP small dataset on small POI when varying epsilon
    std::map<int, input_struct> input_file3{
        {0, input_struct("EP_10062.xyz", "EP_50_poi_on_10062.txt", 0, 1)},
    };

    for (int i = 0; i < input_file3.size(); i++)
    {
        for (int j = 0; j < epsilon_list.size(); j++)
        {
            double epsilon = epsilon_list[j];
            int input_file_index = i;

            run_algorithms(input_file3, output_folder_list[2], epsilon, input_file_index, 1);
        }
    }

    // run EP small dataset on small POI when varying POI number
    std::map<int, input_struct> input_file4{
        {0, input_struct("EP_10062.xyz", "EP_50_poi_on_10062.txt", 0, 1)},
        {1, input_struct("EP_10062.xyz", "EP_100_poi_on_10062.txt", 0, 1)},
        {2, input_struct("EP_10062.xyz", "EP_150_poi_on_10062.txt", 0, 1)},
        {3, input_struct("EP_10062.xyz", "EP_200_poi_on_10062.txt", 0, 1)},
        {4, input_struct("EP_10062.xyz", "EP_250_poi_on_10062.txt", 0, 1)},
    };
    for (int i = 0; i < input_file4.size(); i++)
    {
        double epsilon = 0.1;
        int input_file_index = i;

        run_algorithms(input_file4, output_folder_list[3], epsilon, input_file_index, 1);
    }

    // run EP small dataset on small POI when varying dataset size
    std::map<int, input_struct> input_file5{
        {0, input_struct("EP_10062.xyz", "EP_50_poi_on_10062.txt", 0, 1)},
        {1, input_struct("EP_20130.xyz", "EP_50_poi_on_20130.txt", 0, 1)},
        {2, input_struct("EP_30098.xyz", "EP_50_poi_on_30098.txt", 0, 1)},
        {3, input_struct("EP_40076.xyz", "EP_50_poi_on_40076.txt", 0, 1)},
        {4, input_struct("EP_50373.xyz", "EP_50_poi_on_50373.txt", 0, 1)},
    };
    for (int i = 0; i < input_file5.size(); i++)
    {
        double epsilon = 0.1;
        int input_file_index = i;

        run_algorithms(input_file5, output_folder_list[4], epsilon, input_file_index, 1);
    }

    // run GF small dataset on small POI when varying epsilon
    std::map<int, input_struct> input_file6{
        {0, input_struct("GF_10092.xyz", "GF_50_poi_on_10092.txt", 0, 1)},
    };

    for (int i = 0; i < input_file6.size(); i++)
    {
        for (int j = 0; j < epsilon_list.size(); j++)
        {
            double epsilon = epsilon_list[j];
            int input_file_index = i;

            run_algorithms(input_file6, output_folder_list[5], epsilon, input_file_index, 1);
        }
    }

    // run GF small dataset on small POI when varying POI number
    std::map<int, input_struct> input_file7{
        {0, input_struct("GF_10092.xyz", "GF_50_poi_on_10092.txt", 0, 1)},
        {1, input_struct("GF_10092.xyz", "GF_100_poi_on_10092.txt", 0, 1)},
        {2, input_struct("GF_10092.xyz", "GF_150_poi_on_10092.txt", 0, 1)},
        {3, input_struct("GF_10092.xyz", "GF_200_poi_on_10092.txt", 0, 1)},
        {4, input_struct("GF_10092.xyz", "GF_250_poi_on_10092.txt", 0, 1)},
    };
    for (int i = 0; i < input_file7.size(); i++)
    {
        double epsilon = 0.1;
        int input_file_index = i;

        run_algorithms(input_file7, output_folder_list[6], epsilon, input_file_index, 1);
    }

    // run LM small dataset on small POI when varying epsilon
    std::map<int, input_struct> input_file8{
        {0, input_struct("LM_10092.xyz", "LM_50_poi_on_10092.txt", 0, 1)},
    };

    for (int i = 0; i < input_file8.size(); i++)
    {
        for (int j = 0; j < epsilon_list.size(); j++)
        {
            double epsilon = epsilon_list[j];
            int input_file_index = i;

            run_algorithms(input_file8, output_folder_list[7], epsilon, input_file_index, 1);
        }
    }

    // run LM small dataset on small POI when varying POI number
    std::map<int, input_struct> input_file9{
        {0, input_struct("LM_10092.xyz", "LM_50_poi_on_10092.txt", 0, 1)},
        {1, input_struct("LM_10092.xyz", "LM_100_poi_on_10092.txt", 0, 1)},
        {2, input_struct("LM_10092.xyz", "LM_150_poi_on_10092.txt", 0, 1)},
        {3, input_struct("LM_10092.xyz", "LM_200_poi_on_10092.txt", 0, 1)},
        {4, input_struct("LM_10092.xyz", "LM_250_poi_on_10092.txt", 0, 1)},
    };
    for (int i = 0; i < input_file9.size(); i++)
    {
        double epsilon = 0.1;
        int input_file_index = i;

        run_algorithms(input_file9, output_folder_list[8], epsilon, input_file_index, 1);
    }

    // run RM small dataset on small POI when varying epsilon
    std::map<int, input_struct> input_file10{
        {0, input_struct("RM_10092.xyz", "RM_50_poi_on_10092.txt", 0, 1)},
    };

    for (int i = 0; i < input_file10.size(); i++)
    {
        for (int j = 0; j < epsilon_list.size(); j++)
        {
            double epsilon = epsilon_list[j];
            int input_file_index = i;

            run_algorithms(input_file10, output_folder_list[9], epsilon, input_file_index, 1);
        }
    }

    // run RM small dataset on small POI when varying POI number
    std::map<int, input_struct> input_file11{
        {0, input_struct("RM_10092.xyz", "RM_50_poi_on_10092.txt", 0, 1)},
        {1, input_struct("RM_10092.xyz", "RM_100_poi_on_10092.txt", 0, 1)},
        {2, input_struct("RM_10092.xyz", "RM_150_poi_on_10092.txt", 0, 1)},
        {3, input_struct("RM_10092.xyz", "RM_200_poi_on_10092.txt", 0, 1)},
        {4, input_struct("RM_10092.xyz", "RM_250_poi_on_10092.txt", 0, 1)},
    };
    for (int i = 0; i < input_file11.size(); i++)
    {
        double epsilon = 0.1;
        int input_file_index = i;

        run_algorithms(input_file11, output_folder_list[10], epsilon, input_file_index, 1);
    }

    // run BH dataset on large POI when varying epsilon
    std::map<int, input_struct> input_file12{
        {0, input_struct("BH_500835.xyz", "BH_500_poi_on_500835.txt", 0, 1)},
    };

    for (int i = 0; i < input_file12.size(); i++)
    {
        for (int j = 0; j < epsilon_list.size(); j++)
        {
            double epsilon = epsilon_list[j];
            int input_file_index = i;

            run_algorithms(input_file12, output_folder_list[11], epsilon, input_file_index, 2);
        }
    }

    // run BH dataset on large POI when varying POI number
    std::map<int, input_struct> input_file13{
        {0, input_struct("BH_500835.xyz", "BH_500_poi_on_500835.txt", 0, 1)},
        {1, input_struct("BH_500835.xyz", "BH_1000_poi_on_500835.txt", 0, 1)},
        {2, input_struct("BH_500835.xyz", "BH_1500_poi_on_500835.txt", 0, 1)},
        {3, input_struct("BH_500835.xyz", "BH_2000_poi_on_500835.txt", 0, 1)},
        {4, input_struct("BH_500835.xyz", "BH_2500_poi_on_500835.txt", 0, 1)},
    };
    for (int i = 0; i < input_file13.size(); i++)
    {
        double epsilon = 0.25;
        int input_file_index = i;

        run_algorithms(input_file13, output_folder_list[12], epsilon, input_file_index, 2);
    }

    // run BH dataset on large POI when varying dataset size
    std::map<int, input_struct> input_file14{
        {0, input_struct("BH_500835.xyz", "BH_500_poi_on_500835.txt", 0, 1)},
        {1, input_struct("BH_1000414.xyz", "BH_500_poi_on_1000414.txt", 0, 1)},
        {2, input_struct("BH_1500996.xyz", "BH_500_poi_on_1500996.txt", 0, 1)},
        {3, input_struct("BH_2001610.xyz", "BH_500_poi_on_2001610.txt", 0, 1)},
        {4, input_struct("BH_2502596.xyz", "BH_500_poi_on_2502596.txt", 0, 1)},
    };
    for (int i = 0; i < input_file14.size(); i++)
    {
        double epsilon = 0.25;
        int input_file_index = i;

        run_algorithms(input_file14, output_folder_list[13], epsilon, input_file_index, 2);
    }

    // run EP dataset on large POI when varying epsilon
    std::map<int, input_struct> input_file15{
        {0, input_struct("EP_500384.xyz", "EP_500_poi_on_500384.txt", 0, 1)},
    };

    for (int i = 0; i < input_file15.size(); i++)
    {
        for (int j = 0; j < epsilon_list.size(); j++)
        {
            double epsilon = epsilon_list[j];
            int input_file_index = i;

            run_algorithms(input_file15, output_folder_list[14], epsilon, input_file_index, 2);
        }
    }

    // run EP dataset on large POI when varying POI number
    std::map<int, input_struct> input_file16{
        {0, input_struct("EP_500384.xyz", "EP_500_poi_on_500384.txt", 0, 1)},
        {1, input_struct("EP_500384.xyz", "EP_1000_poi_on_500384.txt", 0, 1)},
        {2, input_struct("EP_500384.xyz", "EP_1500_poi_on_500384.txt", 0, 1)},
        {3, input_struct("EP_500384.xyz", "EP_2000_poi_on_500384.txt", 0, 1)},
        {4, input_struct("EP_500384.xyz", "EP_2500_poi_on_500384.txt", 0, 1)},
    };
    for (int i = 0; i < input_file16.size(); i++)
    {
        double epsilon = 0.25;
        int input_file_index = i;

        run_algorithms(input_file16, output_folder_list[15], epsilon, input_file_index, 2);
    }

    // run EP dataset on large POI when varying dataset size
    std::map<int, input_struct> input_file17{
        {0, input_struct("EP_500384.xyz", "EP_500_poi_on_500384.txt", 0, 1)},
        {1, input_struct("EP_1001040.xyz", "EP_500_poi_on_1001040.txt", 0, 1)},
        {2, input_struct("EP_1501578.xyz", "EP_500_poi_on_1501578.txt", 0, 1)},
        {3, input_struct("EP_2001536.xyz", "EP_500_poi_on_2001536.txt", 0, 1)},
        {4, input_struct("EP_2500560.xyz", "EP_500_poi_on_2500560.txt", 0, 1)},
    };
    for (int i = 0; i < input_file17.size(); i++)
    {
        double epsilon = 0.25;
        int input_file_index = i;

        run_algorithms(input_file17, output_folder_list[16], epsilon, input_file_index, 2);
    }

    // run GF dataset on large POI when varying epsilon
    std::map<int, input_struct> input_file18{
        {0, input_struct("GF_500208.xyz", "GF_500_poi_on_500208.txt", 0, 1)},
    };

    for (int i = 0; i < input_file18.size(); i++)
    {
        for (int j = 0; j < epsilon_list.size(); j++)
        {
            double epsilon = epsilon_list[j];
            int input_file_index = i;

            run_algorithms(input_file18, output_folder_list[17], epsilon, input_file_index, 2);
        }
    }

    // run GF dataset on large POI when varying POI number
    std::map<int, input_struct> input_file19{
        {0, input_struct("GF_500208.xyz", "GF_500_poi_on_500208.txt", 0, 1)},
        {1, input_struct("GF_500208.xyz", "GF_1000_poi_on_500208.txt", 0, 1)},
        {2, input_struct("GF_500208.xyz", "GF_1500_poi_on_500208.txt", 0, 1)},
        {3, input_struct("GF_500208.xyz", "GF_2000_poi_on_500208.txt", 0, 1)},
        {4, input_struct("GF_500208.xyz", "GF_2500_poi_on_500208.txt", 0, 1)},
    };
    for (int i = 0; i < input_file19.size(); i++)
    {
        double epsilon = 0.25;
        int input_file_index = i;

        run_algorithms(input_file19, output_folder_list[18], epsilon, input_file_index, 2);
    }

    // run GF dataset on large POI when varying dataset size
    std::map<int, input_struct> input_file20{
        {0, input_struct("GF_500208.xyz", "GF_500_poi_on_500208.txt", 0, 1)},
        {1, input_struct("GF_1000518.xyz", "GF_500_poi_on_1000518.txt", 0, 1)},
        {2, input_struct("GF_1501668.xyz", "GF_500_poi_on_1501668.txt", 0, 1)},
        {3, input_struct("GF_2000832.xyz", "GF_500_poi_on_2000832.txt", 0, 1)},
        {4, input_struct("GF_2502075.xyz", "GF_500_poi_on_2502075.txt", 0, 1)},
    };
    for (int i = 0; i < input_file20.size(); i++)
    {
        double epsilon = 0.25;
        int input_file_index = i;

        run_algorithms(input_file20, output_folder_list[19], epsilon, input_file_index, 2);
    }

    // run LM dataset on large POI when varying epsilon
    std::map<int, input_struct> input_file21{
        {0, input_struct("LM_500208.xyz", "LM_500_poi_on_500208.txt", 0, 1)},
    };

    for (int i = 0; i < input_file21.size(); i++)
    {
        for (int j = 0; j < epsilon_list.size(); j++)
        {
            double epsilon = epsilon_list[j];
            int input_file_index = i;

            run_algorithms(input_file21, output_folder_list[20], epsilon, input_file_index, 2);
        }
    }

    // run LM dataset on large POI when varying POI number
    std::map<int, input_struct> input_file22{
        {0, input_struct("LM_500208.xyz", "LM_500_poi_on_500208.txt", 0, 1)},
        {1, input_struct("LM_500208.xyz", "LM_1000_poi_on_500208.txt", 0, 1)},
        {2, input_struct("LM_500208.xyz", "LM_1500_poi_on_500208.txt", 0, 1)},
        {3, input_struct("LM_500208.xyz", "LM_2000_poi_on_500208.txt", 0, 1)},
        {4, input_struct("LM_500208.xyz", "LM_2500_poi_on_500208.txt", 0, 1)},
    };
    for (int i = 0; i < input_file22.size(); i++)
    {
        double epsilon = 0.25;
        int input_file_index = i;

        run_algorithms(input_file22, output_folder_list[21], epsilon, input_file_index, 2);
    }

    // run LM dataset on large POI when varying dataset size
    std::map<int, input_struct> input_file23{
        {0, input_struct("LM_500208.xyz", "LM_500_poi_on_500208.txt", 0, 1)},
        {1, input_struct("LM_1000518.xyz", "LM_500_poi_on_1000518.txt", 0, 1)},
        {2, input_struct("LM_1501668.xyz", "LM_500_poi_on_1501668.txt", 0, 1)},
        {3, input_struct("LM_2000832.xyz", "LM_500_poi_on_2000832.txt", 0, 1)},
        {4, input_struct("LM_2502075.xyz", "LM_500_poi_on_2502075.txt", 0, 1)},
    };
    for (int i = 0; i < input_file23.size(); i++)
    {
        double epsilon = 0.25;
        int input_file_index = i;

        run_algorithms(input_file23, output_folder_list[22], epsilon, input_file_index, 2);
    }

    // run RM dataset on large POI when varying epsilon
    std::map<int, input_struct> input_file24{
        {0, input_struct("RM_500208.xyz", "RM_500_poi_on_500208.txt", 0, 1)},
    };

    for (int i = 0; i < input_file24.size(); i++)
    {
        for (int j = 0; j < epsilon_list.size(); j++)
        {
            double epsilon = epsilon_list[j];
            int input_file_index = i;

            run_algorithms(input_file24, output_folder_list[23], epsilon, input_file_index, 2);
        }
    }

    // run RM dataset on large POI when varying POI number
    std::map<int, input_struct> input_file25{
        {0, input_struct("RM_500208.xyz", "RM_500_poi_on_500208.txt", 0, 1)},
        {1, input_struct("RM_500208.xyz", "RM_1000_poi_on_500208.txt", 0, 1)},
        {2, input_struct("RM_500208.xyz", "RM_1500_poi_on_500208.txt", 0, 1)},
        {3, input_struct("RM_500208.xyz", "RM_2000_poi_on_500208.txt", 0, 1)},
        {4, input_struct("RM_500208.xyz", "RM_2500_poi_on_500208.txt", 0, 1)},
    };
    for (int i = 0; i < input_file25.size(); i++)
    {
        double epsilon = 0.25;
        int input_file_index = i;

        run_algorithms(input_file25, output_folder_list[24], epsilon, input_file_index, 2);
    }

    // run RM dataset on large POI when varying dataset size
    std::map<int, input_struct> input_file26{
        {0, input_struct("RM_500208.xyz", "RM_500_poi_on_500208.txt", 0, 1)},
        {1, input_struct("RM_1000518.xyz", "RM_500_poi_on_1000518.txt", 0, 1)},
        {2, input_struct("RM_1501668.xyz", "RM_500_poi_on_1501668.txt", 0, 1)},
        {3, input_struct("RM_2000832.xyz", "RM_500_poi_on_2000832.txt", 0, 1)},
        {4, input_struct("RM_2502075.xyz", "RM_500_poi_on_2502075.txt", 0, 1)},
    };
    for (int i = 0; i < input_file26.size(); i++)
    {
        double epsilon = 0.25;
        int input_file_index = i;

        run_algorithms(input_file26, output_folder_list[25], epsilon, input_file_index, 2);
    }

    // run BH dataset on large POI when varying epsilon for ablation study
    std::map<int, input_struct> input_file27{
        {0, input_struct("BH_500835.xyz", "BH_500_poi_on_500835.txt", 0, 1)},
    };

    for (int i = 0; i < input_file27.size(); i++)
    {
        for (int j = 0; j < epsilon_list.size(); j++)
        {
            double epsilon = epsilon_list[j];
            int input_file_index = i;

            run_algorithms(input_file27, output_folder_list[26], epsilon, input_file_index, 3);
        }
    }

    // run EP dataset on large POI when varying epsilon for ablation study
    std::map<int, input_struct> input_file28{
        {0, input_struct("EP_500384.xyz", "EP_500_poi_on_500384.txt", 0, 1)},
    };

    for (int i = 0; i < input_file28.size(); i++)
    {
        for (int j = 0; j < epsilon_list.size(); j++)
        {
            double epsilon = epsilon_list[j];
            int input_file_index = i;

            run_algorithms(input_file28, output_folder_list[27], epsilon, input_file_index, 3);
        }
    }

    // run GF dataset on large POI when varying epsilon for ablation study
    std::map<int, input_struct> input_file29{
        {0, input_struct("GF_500208.xyz", "GF_500_poi_on_500208.txt", 0, 1)},
    };

    for (int i = 0; i < input_file29.size(); i++)
    {
        for (int j = 0; j < epsilon_list.size(); j++)
        {
            double epsilon = epsilon_list[j];
            int input_file_index = i;

            run_algorithms(input_file29, output_folder_list[28], epsilon, input_file_index, 3);
        }
    }

    // run LM dataset on large POI when varying epsilon for ablation study
    std::map<int, input_struct> input_file30{
        {0, input_struct("LM_500208.xyz", "LM_500_poi_on_500208.txt", 0, 1)},
    };

    for (int i = 0; i < input_file30.size(); i++)
    {
        for (int j = 0; j < epsilon_list.size(); j++)
        {
            double epsilon = epsilon_list[j];
            int input_file_index = i;

            run_algorithms(input_file30, output_folder_list[29], epsilon, input_file_index, 3);
        }
    }

    // run RM dataset on large POI when varying epsilon for ablation study
    std::map<int, input_struct> input_file31{
        {0, input_struct("RM_500208.xyz", "RM_500_poi_on_500208.txt", 0, 1)},
    };

    for (int i = 0; i < input_file31.size(); i++)
    {
        for (int j = 0; j < epsilon_list.size(); j++)
        {
            double epsilon = epsilon_list[j];
            int input_file_index = i;

            run_algorithms(input_file31, output_folder_list[30], epsilon, input_file_index, 3);
        }
    }

    // run BH small dataset on small POI when varying epsilon for proximity query oracle and algorithm comparision
    std::map<int, input_struct> input_file32{
        {0, input_struct("BH_10086.xyz", "BH_50_poi_on_10086.txt", 0, 1)},
    };

    for (int i = 0; i < input_file32.size(); i++)
    {
        for (int j = 0; j < epsilon_list.size(); j++)
        {
            double epsilon = epsilon_list[j];
            int input_file_index = i;

            run_algorithms(input_file32, output_folder_list[31], epsilon, input_file_index, 4);
        }
    }

    // run EP small dataset on small POI when varying epsilon for proximity query oracle and algorithm comparision
    std::map<int, input_struct> input_file33{
        {0, input_struct("EP_10062.xyz", "EP_50_poi_on_10062.txt", 0, 1)},
    };

    for (int i = 0; i < input_file33.size(); i++)
    {
        for (int j = 0; j < epsilon_list.size(); j++)
        {
            double epsilon = epsilon_list[j];
            int input_file_index = i;

            run_algorithms(input_file33, output_folder_list[32], epsilon, input_file_index, 4);
        }
    }

    // run GF small dataset on small POI when varying epsilon for proximity query oracle and algorithm comparision
    std::map<int, input_struct> input_file34{
        {0, input_struct("GF_10092.xyz", "GF_50_poi_on_10092.txt", 0, 1)},
    };

    for (int i = 0; i < input_file34.size(); i++)
    {
        for (int j = 0; j < epsilon_list.size(); j++)
        {
            double epsilon = epsilon_list[j];
            int input_file_index = i;

            run_algorithms(input_file34, output_folder_list[33], epsilon, input_file_index, 4);
        }
    }

    // run LM small dataset on small POI when varying epsilon for proximity query oracle and algorithm comparision
    std::map<int, input_struct> input_file35{
        {0, input_struct("LM_10092.xyz", "LM_50_poi_on_10092.txt", 0, 1)},
    };

    for (int i = 0; i < input_file35.size(); i++)
    {
        for (int j = 0; j < epsilon_list.size(); j++)
        {
            double epsilon = epsilon_list[j];
            int input_file_index = i;

            run_algorithms(input_file35, output_folder_list[34], epsilon, input_file_index, 4);
        }
    }

    // run RM small dataset on small POI when varying epsilon for proximity query oracle and algorithm comparision
    std::map<int, input_struct> input_file36{
        {0, input_struct("RM_10092.xyz", "RM_50_poi_on_10092.txt", 0, 1)},
    };

    for (int i = 0; i < input_file36.size(); i++)
    {
        for (int j = 0; j < epsilon_list.size(); j++)
        {
            double epsilon = epsilon_list[j];
            int input_file_index = i;

            run_algorithms(input_file36, output_folder_list[35], epsilon, input_file_index, 4);
        }
    }
}
