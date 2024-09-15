#include "algorithms.h"
#include <filesystem>

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

void run_algorithms(std::map<int, input_struct> input_file,
                    std::string output_folder,
                    double e, int input_file_index, int algorithm_type)
{
    int k_value = 3;
    double range = 100;
    bool run_knn_query = false;
    bool run_range_query = false;

    std::string input_folder = "../input/";
    std::string input_poi_folder = "../input/";

    std::string input_point_cloud = input_folder + input_file[input_file_index].point_cloud_or_terrain + ".xyz";
    std::string input_terrain = input_folder + input_file[input_file_index].point_cloud_or_terrain + ".off";
    std::string input_poi = input_poi_folder + input_file[input_file_index].poi;
    int source_poi_index = input_file[input_file_index].source_poi_index;
    int destination_poi_index = input_file[input_file_index].destination_poi_index;
    assert(source_poi_index != destination_poi_index);

    std::cout << "============" << std::endl;

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

    if (algorithm_type == 1)
    {
        std::cout << "== SE_Oracle_Adapt ==" << std::endl;
        SE_Oracle_or_SE_Oracle_Adapt_with_output(output_folder + "1_SE_Oracle_Adapt.txt", poi_num, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                                 terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                                 terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                                 write_file_header, 1);
        std::cout << std::endl;
    }

    if (algorithm_type == 1 || algorithm_type == 5)
    {
        std::cout << "== EAR_Oracle_Adapt ==" << std::endl;
        EAR_Oracle_or_EAR_Oracle_Adapt_with_output(output_folder + "2_EAR_Oracle_Adapt.txt", poi_num, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                                   terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                                   terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                                   write_file_header, 1);
        std::cout << std::endl;
    }

    if (algorithm_type == 1)
    {
        std::cout << "== RC_Oracle_Naive ==" << std::endl;
        RC_Oracle_Naive_or_RC_Oracle_Naive_Adapt_with_output(output_folder + "3_RC_Oracle_Naive.txt", poi_num, &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                                             terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                                             terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                                             write_file_header, 1);
        std::cout << std::endl;
    }

    if (algorithm_type == 1 || algorithm_type == 2 || algorithm_type == 3 || algorithm_type == 4)
    {
        std::cout << "== RC_Oracle ==" << std::endl;
        RC_Oracle_or_RC_Oracle_Adapt_with_output(output_folder + "4_RC_Oracle.txt", poi_num, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                                 terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                                 terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                                 write_file_header, 1);
        std::cout << std::endl;
    }

    if (algorithm_type == 1 || algorithm_type == 2 || algorithm_type == 5)
    {
        std::cout << "== CH_Adapt ==" << std::endl;
        CH_or_CH_Adapt_with_output(output_folder + "5_CH_Adapt.txt", &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                   terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                   terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                   write_file_header, 1);
        std::cout << std::endl;

        std::cout << "== Kaul_Adapt ==" << std::endl;
        Kaul_or_Kaul_Adapt_with_output(output_folder + "6_Kaul_Adapt.txt", &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                       terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                       terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                       write_file_header, 1);
        std::cout << std::endl;

        std::cout << "== Dijk_Adapt ==" << std::endl;
        Dijk_or_Dijk_Adapt_with_output(output_folder + "7_Dijk_Adapt.txt", &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                       terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                       terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                       write_file_header, 1);
        std::cout << std::endl;

        std::cout << "== FastFly ==" << std::endl;
        FastFly_or_FastFly_Adapt_with_output(output_folder + "8_FastFly.txt", &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                             terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                             terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                             write_file_header, 1);
        std::cout << std::endl;
    }

    if (algorithm_type == 3)
    {
        std::cout << "== SE_Oracle_FastFly_Adapt ==" << std::endl;
        SE_Oracle_FastFly_Adapt_with_output(
            output_folder + "9_SE_Oracle_FastFly_Adapt.txt", poi_num, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
            terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
            terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
            write_file_header);
        std::cout << std::endl;
    }

    if (algorithm_type == 3 || algorithm_type == 6)
    {
        std::cout << "== EAR_Oracle_FastFly_Adapt ==" << std::endl;
        EAR_Oracle_FastFly_Adapt_with_output(
            output_folder + "10_EAR_Oracle_FastFly_Adapt.txt", poi_num, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
            terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
            terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
            write_file_header);
        std::cout << std::endl;
    }

    if (algorithm_type == 4 || algorithm_type == 7)
    {
        std::cout << "== SU_Oracle_Adapt ==" << std::endl;
        SU_Oracle_Adapt_with_output(output_folder + "11_SU_Oracle_Adapt.txt", poi_num, &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                    terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                    terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                    write_file_header);
        std::cout << std::endl;
    }

    if (algorithm_type == 4)
    {
        std::cout << "== RC_Oracle_NaiveProx ==" << std::endl;
        RC_Oracle_NaiveProx_with_output(output_folder + "12_RC_Oracle_NaiveProx.txt", poi_num, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                        terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                        terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                        write_file_header);
        std::cout << std::endl;
    }

    if (algorithm_type == 5)
    {
        std::cout << "== SE_Oracle_Adapt_A2A ==" << std::endl;
        SE_Oracle_Adapt_A2A_with_output(output_folder + "13_SE_Oracle_Adapt_A2A.txt", poi_num, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                        terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                        terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                        write_file_header);
        std::cout << std::endl;

        std::cout << "== RC_Oracle_Naive_A2A ==" << std::endl;
        RC_Oracle_Naive_A2A_with_output(output_folder + "14_RC_Oracle_Naive_A2A.txt", poi_num, &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                        terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                        terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                        write_file_header);
        std::cout << std::endl;
    }

    if (algorithm_type == 5 || algorithm_type == 6 || algorithm_type == 7)
    {
        std::cout << "== RC_Oracle_A2A ==" << std::endl;
        RC_Oracle_A2A_with_output(output_folder + "15_RC_Oracle_A2A.txt", poi_num, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                  terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                  terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                  write_file_header);
        std::cout << std::endl;
    }

    if (algorithm_type == 6)
    {
        std::cout << "== SE_Oracle_FastFly_Adapt_A2A ==" << std::endl;
        SE_Oracle_FastFly_Adapt_A2A_with_output(output_folder + "16_SE_Oracle_FastFly_Adapt_A2A.txt", poi_num, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                                terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                                terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                                write_file_header);
        std::cout << std::endl;
    }

    if (algorithm_type == 7)
    {
        std::cout << "== RC_Oracle_NaiveProx_A2A ==" << std::endl;
        RC_Oracle_NaiveProx_A2A_with_output(output_folder + "17_RC_Oracle_NaiveProx_A2A.txt", poi_num, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                            terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                            terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                            write_file_header);
        std::cout << std::endl;
    }

    if (algorithm_type == 8)
    {
        std::cout << "== SE_Oracle ==" << std::endl;
        SE_Oracle_or_SE_Oracle_Adapt_with_output(output_folder + "18_SE_Oracle.txt", poi_num, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                                 terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                                 terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                                 write_file_header, 2);
        std::cout << std::endl;

        std::cout << "== EAR_Oracle ==" << std::endl;
        EAR_Oracle_or_EAR_Oracle_Adapt_with_output(output_folder + "19_EAR_Oracle.txt", poi_num, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                                   terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                                   terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                                   write_file_header, 2);
        std::cout << std::endl;

        std::cout << "== RC_Oracle_Naive_Adapt ==" << std::endl;
        RC_Oracle_Naive_or_RC_Oracle_Naive_Adapt_with_output(output_folder + "20_RC_Oracle_Naive_Adapt.txt", poi_num, &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                                             terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                                             terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                                             write_file_header, 2);
        std::cout << std::endl;
    }

    if (algorithm_type == 8 || algorithm_type == 9)
    {
        std::cout << "== RC_Oracle_Adapt ==" << std::endl;
        RC_Oracle_or_RC_Oracle_Adapt_with_output(output_folder + "21_RC_Oracle_Adapt.txt", poi_num, &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                                 terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                                 terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                                 write_file_header, 2);
        std::cout << std::endl;

        std::cout << "== CH ==" << std::endl;
        CH_or_CH_Adapt_with_output(output_folder + "22_CH.txt", &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                   terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                   terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                   write_file_header, 2);
        std::cout << std::endl;

        std::cout << "== Kaul ==" << std::endl;
        Kaul_or_Kaul_Adapt_with_output(output_folder + "23_Kaul.txt", &point_cloud, poi_list, e, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                       terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                       terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                       write_file_header, 2);
        std::cout << std::endl;

        std::cout << "== Dijk ==" << std::endl;
        Dijk_or_Dijk_Adapt_with_output(output_folder + "24_Dijk.txt", &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                       terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                       terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                       write_file_header, 2);
        std::cout << std::endl;

        std::cout << "== FastFly_Adapt ==" << std::endl;
        FastFly_or_FastFly_Adapt_with_output(output_folder + "25_FastFly_Adapt.txt", &point_cloud, poi_list, source_poi_index, destination_poi_index, point_cloud_exact_distance,
                                             terrain_exact_distance, run_knn_query, run_range_query, k_value, range, point_cloud_exact_all_poi_knn_query_list,
                                             terrain_exact_all_poi_knn_query_list, point_cloud_exact_all_poi_range_query_list, terrain_exact_all_poi_range_query_list,
                                             write_file_header, 2);
        std::cout << std::endl;
    }

    input.close();
}

int main(int argc, char **argv)
{
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
        "../exp/36_e_RM_small_proxim_ora/",
        "../exp/37_e_BH_small_A2A/",
        "../exp/38_e_EP_small_A2A/",
        "../exp/39_e_GF_small_A2A/",
        "../exp/40_e_LM_small_A2A/",
        "../exp/41_e_RM_small_A2A/",
        "../exp/42_e_BH_abla_A2A/",
        "../exp/43_e_EP_abla_A2A/",
        "../exp/44_e_GF_abla_A2A/",
        "../exp/45_e_LM_abla_A2A/",
        "../exp/46_e_RM_abla_A2A/",
        "../exp/47_e_BH_small_proxim_ora_A2A/",
        "../exp/48_e_EP_small_proxim_ora_A2A/",
        "../exp/49_e_GF_small_proxim_ora_A2A/",
        "../exp/50_e_LM_small_proxim_ora_A2A/",
        "../exp/51_e_RM_small_proxim_ora_A2A/",
        "../exp/52_e_BH_small_TIN/",
        "../exp/53_n_BH_small_TIN/",
        "../exp/54_e_EP_small_TIN/",
        "../exp/55_n_EP_small_TIN/",
        "../exp/56_DS_EP_small_TIN/",
        "../exp/57_e_GF_small_TIN/",
        "../exp/58_n_GF_small_TIN/",
        "../exp/59_e_LM_small_TIN/",
        "../exp/60_n_LM_small_TIN/",
        "../exp/61_e_RM_small_TIN/",
        "../exp/62_n_RM_small_TIN/",
        "../exp/63_e_BH_TIN/",
        "../exp/64_n_BH_TIN/",
        "../exp/65_DS_BH_TIN/",
        "../exp/66_e_EP_TIN/",
        "../exp/67_n_EP_TIN/",
        "../exp/68_DS_EP_TIN/",
        "../exp/69_e_GF_TIN/",
        "../exp/70_n_GF_TIN/",
        "../exp/71_DS_GF_TIN/",
        "../exp/72_e_LM_TIN/",
        "../exp/73_n_LM_TIN/",
        "../exp/74_DS_LM_TIN/",
        "../exp/75_e_RM_TIN/",
        "../exp/76_n_RM_TIN/",
        "../exp/77_DS_RM_TIN/"};
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
                                                 "12_RC_Oracle_NaiveProx.txt",
                                                 "13_SE_Oracle_Adapt_A2A.txt",
                                                 "14_RC_Oracle_Naive_A2A.txt",
                                                 "15_RC_Oracle_A2A.txt",
                                                 "16_SE_Oracle_FastFly_Adapt_A2A.txt",
                                                 "17_RC_Oracle_NaiveProx_A2A.txt",
                                                 "18_SE_Oracle.txt",
                                                 "19_EAR_Oracle.txt",
                                                 "20_RC_Oracle_Naive_Adapt.txt",
                                                 "21_RC_Oracle_Adapt.txt",
                                                 "22_CH.txt",
                                                 "23_Kaul.txt",
                                                 "24_Dijk.txt",
                                                 "25_FastFly_Adapt.txt"};
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

    std::vector<double> e_list = {0.05, 0.1, 0.25, 0.5, 0.75, 1};

    // run BH small dataset on small POI when varying e
    std::map<int, input_struct> input_file1{
        {0, input_struct("BH_10086", "BH_50_poi_on_10086.txt", 0, 1)},
    };
    for (int i = 0; i < input_file1.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file1, output_folder_list[0], e, input_file_index, 1);
        }
    }

    // run BH small dataset on small POI when varying POI number
    std::map<int, input_struct> input_file2{
        {0, input_struct("BH_10086", "BH_50_poi_on_10086.txt", 0, 1)},
        {1, input_struct("BH_10086", "BH_100_poi_on_10086.txt", 0, 1)},
        {2, input_struct("BH_10086", "BH_150_poi_on_10086.txt", 0, 1)},
        {3, input_struct("BH_10086", "BH_200_poi_on_10086.txt", 0, 1)},
        {4, input_struct("BH_10086", "BH_250_poi_on_10086.txt", 0, 1)},
    };
    for (int i = 0; i < input_file2.size(); i++)
    {
        double e = 0.1;
        int input_file_index = i;

        run_algorithms(input_file2, output_folder_list[1], e, input_file_index, 1);
    }

    // run EP small dataset on small POI when varying e
    std::map<int, input_struct> input_file3{
        {0, input_struct("EP_10062", "EP_50_poi_on_10062.txt", 0, 1)},
    };

    for (int i = 0; i < input_file3.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file3, output_folder_list[2], e, input_file_index, 1);
        }
    }

    // run EP small dataset on small POI when varying POI number
    std::map<int, input_struct> input_file4{
        {0, input_struct("EP_10062", "EP_50_poi_on_10062.txt", 0, 1)},
        {1, input_struct("EP_10062", "EP_100_poi_on_10062.txt", 0, 1)},
        {2, input_struct("EP_10062", "EP_150_poi_on_10062.txt", 0, 1)},
        {3, input_struct("EP_10062", "EP_200_poi_on_10062.txt", 0, 1)},
        {4, input_struct("EP_10062", "EP_250_poi_on_10062.txt", 0, 1)},
    };
    for (int i = 0; i < input_file4.size(); i++)
    {
        double e = 0.1;
        int input_file_index = i;

        run_algorithms(input_file4, output_folder_list[3], e, input_file_index, 1);
    }

    // run EP small dataset on small POI when varying dataset size
    std::map<int, input_struct> input_file5{
        {0, input_struct("EP_10062", "EP_50_poi_on_10062.txt", 0, 1)},
        {1, input_struct("EP_20130", "EP_50_poi_on_20130.txt", 0, 1)},
        {2, input_struct("EP_30098", "EP_50_poi_on_30098.txt", 0, 1)},
        {3, input_struct("EP_40076", "EP_50_poi_on_40076.txt", 0, 1)},
        {4, input_struct("EP_50373", "EP_50_poi_on_50373.txt", 0, 1)},
    };
    for (int i = 0; i < input_file5.size(); i++)
    {
        double e = 0.1;
        int input_file_index = i;

        run_algorithms(input_file5, output_folder_list[4], e, input_file_index, 1);
    }

    // run GF small dataset on small POI when varying e
    std::map<int, input_struct> input_file6{
        {0, input_struct("GF_10092", "GF_50_poi_on_10092.txt", 0, 1)},
    };

    for (int i = 0; i < input_file6.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file6, output_folder_list[5], e, input_file_index, 1);
        }
    }

    // run GF small dataset on small POI when varying POI number
    std::map<int, input_struct> input_file7{
        {0, input_struct("GF_10092", "GF_50_poi_on_10092.txt", 0, 1)},
        {1, input_struct("GF_10092", "GF_100_poi_on_10092.txt", 0, 1)},
        {2, input_struct("GF_10092", "GF_150_poi_on_10092.txt", 0, 1)},
        {3, input_struct("GF_10092", "GF_200_poi_on_10092.txt", 0, 1)},
        {4, input_struct("GF_10092", "GF_250_poi_on_10092.txt", 0, 1)},
    };
    for (int i = 0; i < input_file7.size(); i++)
    {
        double e = 0.1;
        int input_file_index = i;

        run_algorithms(input_file7, output_folder_list[6], e, input_file_index, 1);
    }

    // run LM small dataset on small POI when varying e
    std::map<int, input_struct> input_file8{
        {0, input_struct("LM_10092", "LM_50_poi_on_10092.txt", 0, 1)},
    };

    for (int i = 0; i < input_file8.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file8, output_folder_list[7], e, input_file_index, 1);
        }
    }

    // run LM small dataset on small POI when varying POI number
    std::map<int, input_struct> input_file9{
        {0, input_struct("LM_10092", "LM_50_poi_on_10092.txt", 0, 1)},
        {1, input_struct("LM_10092", "LM_100_poi_on_10092.txt", 0, 1)},
        {2, input_struct("LM_10092", "LM_150_poi_on_10092.txt", 0, 1)},
        {3, input_struct("LM_10092", "LM_200_poi_on_10092.txt", 0, 1)},
        {4, input_struct("LM_10092", "LM_250_poi_on_10092.txt", 0, 1)},
    };
    for (int i = 0; i < input_file9.size(); i++)
    {
        double e = 0.1;
        int input_file_index = i;

        run_algorithms(input_file9, output_folder_list[8], e, input_file_index, 1);
    }

    // run RM small dataset on small POI when varying e
    std::map<int, input_struct> input_file10{
        {0, input_struct("RM_10092", "RM_50_poi_on_10092.txt", 0, 1)},
    };

    for (int i = 0; i < input_file10.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file10, output_folder_list[9], e, input_file_index, 1);
        }
    }

    // run RM small dataset on small POI when varying POI number
    std::map<int, input_struct> input_file11{
        {0, input_struct("RM_10092", "RM_50_poi_on_10092.txt", 0, 1)},
        {1, input_struct("RM_10092", "RM_100_poi_on_10092.txt", 0, 1)},
        {2, input_struct("RM_10092", "RM_150_poi_on_10092.txt", 0, 1)},
        {3, input_struct("RM_10092", "RM_200_poi_on_10092.txt", 0, 1)},
        {4, input_struct("RM_10092", "RM_250_poi_on_10092.txt", 0, 1)},
    };
    for (int i = 0; i < input_file11.size(); i++)
    {
        double e = 0.1;
        int input_file_index = i;

        run_algorithms(input_file11, output_folder_list[10], e, input_file_index, 1);
    }

    // run BH dataset on large POI when varying e
    std::map<int, input_struct> input_file12{
        {0, input_struct("BH_500835", "BH_500_poi_on_500835.txt", 0, 1)},
    };

    for (int i = 0; i < input_file12.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file12, output_folder_list[11], e, input_file_index, 2);
        }
    }

    // run BH dataset on large POI when varying POI number
    std::map<int, input_struct> input_file13{
        {0, input_struct("BH_500835", "BH_500_poi_on_500835.txt", 0, 1)},
        {1, input_struct("BH_500835", "BH_1000_poi_on_500835.txt", 0, 1)},
        {2, input_struct("BH_500835", "BH_1500_poi_on_500835.txt", 0, 1)},
        {3, input_struct("BH_500835", "BH_2000_poi_on_500835.txt", 0, 1)},
        {4, input_struct("BH_500835", "BH_2500_poi_on_500835.txt", 0, 1)},
    };
    for (int i = 0; i < input_file13.size(); i++)
    {
        double e = 0.25;
        int input_file_index = i;

        run_algorithms(input_file13, output_folder_list[12], e, input_file_index, 2);
    }

    // run BH dataset on large POI when varying dataset size
    std::map<int, input_struct> input_file14{
        {0, input_struct("BH_500835", "BH_500_poi_on_500835.txt", 0, 1)},
        {1, input_struct("BH_1000414", "BH_500_poi_on_1000414.txt", 0, 1)},
        {2, input_struct("BH_1500996", "BH_500_poi_on_1500996.txt", 0, 1)},
        {3, input_struct("BH_2001610", "BH_500_poi_on_2001610.txt", 0, 1)},
        {4, input_struct("BH_2502596", "BH_500_poi_on_2502596.txt", 0, 1)},
    };
    for (int i = 0; i < input_file14.size(); i++)
    {
        double e = 0.25;
        int input_file_index = i;

        run_algorithms(input_file14, output_folder_list[13], e, input_file_index, 2);
    }

    // run EP dataset on large POI when varying e
    std::map<int, input_struct> input_file15{
        {0, input_struct("EP_500384", "EP_500_poi_on_500384.txt", 0, 1)},
    };

    for (int i = 0; i < input_file15.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file15, output_folder_list[14], e, input_file_index, 2);
        }
    }

    // run EP dataset on large POI when varying POI number
    std::map<int, input_struct> input_file16{
        {0, input_struct("EP_500384", "EP_500_poi_on_500384.txt", 0, 1)},
        {1, input_struct("EP_500384", "EP_1000_poi_on_500384.txt", 0, 1)},
        {2, input_struct("EP_500384", "EP_1500_poi_on_500384.txt", 0, 1)},
        {3, input_struct("EP_500384", "EP_2000_poi_on_500384.txt", 0, 1)},
        {4, input_struct("EP_500384", "EP_2500_poi_on_500384.txt", 0, 1)},
    };
    for (int i = 0; i < input_file16.size(); i++)
    {
        double e = 0.25;
        int input_file_index = i;

        run_algorithms(input_file16, output_folder_list[15], e, input_file_index, 2);
    }

    // run EP dataset on large POI when varying dataset size
    std::map<int, input_struct> input_file17{
        {0, input_struct("EP_500384", "EP_500_poi_on_500384.txt", 0, 1)},
        {1, input_struct("EP_1001040", "EP_500_poi_on_1001040.txt", 0, 1)},
        {2, input_struct("EP_1501578", "EP_500_poi_on_1501578.txt", 0, 1)},
        {3, input_struct("EP_2001536", "EP_500_poi_on_2001536.txt", 0, 1)},
        {4, input_struct("EP_2500560", "EP_500_poi_on_2500560.txt", 0, 1)},
    };
    for (int i = 0; i < input_file17.size(); i++)
    {
        double e = 0.25;
        int input_file_index = i;

        run_algorithms(input_file17, output_folder_list[16], e, input_file_index, 2);
    }

    // run GF dataset on large POI when varying e
    std::map<int, input_struct> input_file18{
        {0, input_struct("GF_500208", "GF_500_poi_on_500208.txt", 0, 1)},
    };

    for (int i = 0; i < input_file18.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file18, output_folder_list[17], e, input_file_index, 2);
        }
    }

    // run GF dataset on large POI when varying POI number
    std::map<int, input_struct> input_file19{
        {0, input_struct("GF_500208", "GF_500_poi_on_500208.txt", 0, 1)},
        {1, input_struct("GF_500208", "GF_1000_poi_on_500208.txt", 0, 1)},
        {2, input_struct("GF_500208", "GF_1500_poi_on_500208.txt", 0, 1)},
        {3, input_struct("GF_500208", "GF_2000_poi_on_500208.txt", 0, 1)},
        {4, input_struct("GF_500208", "GF_2500_poi_on_500208.txt", 0, 1)},
    };
    for (int i = 0; i < input_file19.size(); i++)
    {
        double e = 0.25;
        int input_file_index = i;

        run_algorithms(input_file19, output_folder_list[18], e, input_file_index, 2);
    }

    // run GF dataset on large POI when varying dataset size
    std::map<int, input_struct> input_file20{
        {0, input_struct("GF_500208", "GF_500_poi_on_500208.txt", 0, 1)},
        {1, input_struct("GF_1000518", "GF_500_poi_on_1000518.txt", 0, 1)},
        {2, input_struct("GF_1501668", "GF_500_poi_on_1501668.txt", 0, 1)},
        {3, input_struct("GF_2000832", "GF_500_poi_on_2000832.txt", 0, 1)},
        {4, input_struct("GF_2502075", "GF_500_poi_on_2502075.txt", 0, 1)},
    };
    for (int i = 0; i < input_file20.size(); i++)
    {
        double e = 0.25;
        int input_file_index = i;

        run_algorithms(input_file20, output_folder_list[19], e, input_file_index, 2);
    }

    // run LM dataset on large POI when varying e
    std::map<int, input_struct> input_file21{
        {0, input_struct("LM_500208", "LM_500_poi_on_500208.txt", 0, 1)},
    };

    for (int i = 0; i < input_file21.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file21, output_folder_list[20], e, input_file_index, 2);
        }
    }

    // run LM dataset on large POI when varying POI number
    std::map<int, input_struct> input_file22{
        {0, input_struct("LM_500208", "LM_500_poi_on_500208.txt", 0, 1)},
        {1, input_struct("LM_500208", "LM_1000_poi_on_500208.txt", 0, 1)},
        {2, input_struct("LM_500208", "LM_1500_poi_on_500208.txt", 0, 1)},
        {3, input_struct("LM_500208", "LM_2000_poi_on_500208.txt", 0, 1)},
        {4, input_struct("LM_500208", "LM_2500_poi_on_500208.txt", 0, 1)},
    };
    for (int i = 0; i < input_file22.size(); i++)
    {
        double e = 0.25;
        int input_file_index = i;

        run_algorithms(input_file22, output_folder_list[21], e, input_file_index, 2);
    }

    // run LM dataset on large POI when varying dataset size
    std::map<int, input_struct> input_file23{
        {0, input_struct("LM_500208", "LM_500_poi_on_500208.txt", 0, 1)},
        {1, input_struct("LM_1000518", "LM_500_poi_on_1000518.txt", 0, 1)},
        {2, input_struct("LM_1501668", "LM_500_poi_on_1501668.txt", 0, 1)},
        {3, input_struct("LM_2000832", "LM_500_poi_on_2000832.txt", 0, 1)},
        {4, input_struct("LM_2502075", "LM_500_poi_on_2502075.txt", 0, 1)},
    };
    for (int i = 0; i < input_file23.size(); i++)
    {
        double e = 0.25;
        int input_file_index = i;

        run_algorithms(input_file23, output_folder_list[22], e, input_file_index, 2);
    }

    // run RM dataset on large POI when varying e
    std::map<int, input_struct> input_file24{
        {0, input_struct("RM_500208", "RM_500_poi_on_500208.txt", 0, 1)},
    };

    for (int i = 0; i < input_file24.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file24, output_folder_list[23], e, input_file_index, 2);
        }
    }

    // run RM dataset on large POI when varying POI number
    std::map<int, input_struct> input_file25{
        {0, input_struct("RM_500208", "RM_500_poi_on_500208.txt", 0, 1)},
        {1, input_struct("RM_500208", "RM_1000_poi_on_500208.txt", 0, 1)},
        {2, input_struct("RM_500208", "RM_1500_poi_on_500208.txt", 0, 1)},
        {3, input_struct("RM_500208", "RM_2000_poi_on_500208.txt", 0, 1)},
        {4, input_struct("RM_500208", "RM_2500_poi_on_500208.txt", 0, 1)},
    };
    for (int i = 0; i < input_file25.size(); i++)
    {
        double e = 0.25;
        int input_file_index = i;

        run_algorithms(input_file25, output_folder_list[24], e, input_file_index, 2);
    }

    // run RM dataset on large POI when varying dataset size
    std::map<int, input_struct> input_file26{
        {0, input_struct("RM_500208", "RM_500_poi_on_500208.txt", 0, 1)},
        {1, input_struct("RM_1000518", "RM_500_poi_on_1000518.txt", 0, 1)},
        {2, input_struct("RM_1501668", "RM_500_poi_on_1501668.txt", 0, 1)},
        {3, input_struct("RM_2000832", "RM_500_poi_on_2000832.txt", 0, 1)},
        {4, input_struct("RM_2502075", "RM_500_poi_on_2502075.txt", 0, 1)},
    };
    for (int i = 0; i < input_file26.size(); i++)
    {
        double e = 0.25;
        int input_file_index = i;

        run_algorithms(input_file26, output_folder_list[25], e, input_file_index, 2);
    }

    // run BH dataset on large POI when varying e for ablation study
    std::map<int, input_struct> input_file27{
        {0, input_struct("BH_500835", "BH_500_poi_on_500835.txt", 0, 1)},
    };

    for (int i = 0; i < input_file27.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file27, output_folder_list[26], e, input_file_index, 3);
        }
    }

    // run EP dataset on large POI when varying e for ablation study
    std::map<int, input_struct> input_file28{
        {0, input_struct("EP_500384", "EP_500_poi_on_500384.txt", 0, 1)},
    };

    for (int i = 0; i < input_file28.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file28, output_folder_list[27], e, input_file_index, 3);
        }
    }

    // run GF dataset on large POI when varying e for ablation study
    std::map<int, input_struct> input_file29{
        {0, input_struct("GF_500208", "GF_500_poi_on_500208.txt", 0, 1)},
    };

    for (int i = 0; i < input_file29.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file29, output_folder_list[28], e, input_file_index, 3);
        }
    }

    // run LM dataset on large POI when varying e for ablation study
    std::map<int, input_struct> input_file30{
        {0, input_struct("LM_500208", "LM_500_poi_on_500208.txt", 0, 1)},
    };

    for (int i = 0; i < input_file30.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file30, output_folder_list[29], e, input_file_index, 3);
        }
    }

    // run RM dataset on large POI when varying e for ablation study
    std::map<int, input_struct> input_file31{
        {0, input_struct("RM_500208", "RM_500_poi_on_500208.txt", 0, 1)},
    };

    for (int i = 0; i < input_file31.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file31, output_folder_list[30], e, input_file_index, 3);
        }
    }

    // run BH small dataset on small POI when varying e for proximity query oracle and algorithm comparision
    std::map<int, input_struct> input_file32{
        {0, input_struct("BH_10086", "BH_50_poi_on_10086.txt", 0, 1)},
    };

    for (int i = 0; i < input_file32.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file32, output_folder_list[31], e, input_file_index, 4);
        }
    }

    // run EP small dataset on small POI when varying e for proximity query oracle and algorithm comparision
    std::map<int, input_struct> input_file33{
        {0, input_struct("EP_10062", "EP_50_poi_on_10062.txt", 0, 1)},
    };

    for (int i = 0; i < input_file33.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file33, output_folder_list[32], e, input_file_index, 4);
        }
    }

    // run GF small dataset on small POI when varying e for proximity query oracle and algorithm comparision
    std::map<int, input_struct> input_file34{
        {0, input_struct("GF_10092", "GF_50_poi_on_10092.txt", 0, 1)},
    };

    for (int i = 0; i < input_file34.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file34, output_folder_list[33], e, input_file_index, 4);
        }
    }

    // run LM small dataset on small POI when varying e for proximity query oracle and algorithm comparision
    std::map<int, input_struct> input_file35{
        {0, input_struct("LM_10092", "LM_50_poi_on_10092.txt", 0, 1)},
    };

    for (int i = 0; i < input_file35.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file35, output_folder_list[34], e, input_file_index, 4);
        }
    }

    // run RM small dataset on small POI when varying e for proximity query oracle and algorithm comparision
    std::map<int, input_struct> input_file36{
        {0, input_struct("RM_10092", "RM_50_poi_on_10092.txt", 0, 1)},
    };

    for (int i = 0; i < input_file36.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file36, output_folder_list[35], e, input_file_index, 4);
        }
    }

    // run BH small dataset on small POI when varying e for A2A query
    std::map<int, input_struct> input_file37{
        {0, input_struct("BH_10086", "BH_50_poi_on_10086.txt", 0, 1)},
    };

    for (int i = 0; i < input_file37.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file37, output_folder_list[36], e, input_file_index, 5);
        }
    }

    // run EP small dataset on small POI when varying e for A2A query
    std::map<int, input_struct> input_file38{
        {0, input_struct("EP_10062", "EP_50_poi_on_10062.txt", 0, 1)},
    };

    for (int i = 0; i < input_file38.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file38, output_folder_list[37], e, input_file_index, 5);
        }
    }

    // run GF small dataset on small POI when varying e for A2A query
    std::map<int, input_struct> input_file39{
        {0, input_struct("GF_10092", "GF_50_poi_on_10092.txt", 0, 1)},
    };

    for (int i = 0; i < input_file39.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file39, output_folder_list[38], e, input_file_index, 5);
        }
    }

    // run LM small dataset on small POI when varying e for A2A query
    std::map<int, input_struct> input_file40{
        {0, input_struct("LM_10092", "LM_50_poi_on_10092.txt", 0, 1)},
    };

    for (int i = 0; i < input_file40.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file40, output_folder_list[39], e, input_file_index, 5);
        }
    }

    // run RM small dataset on small POI when varying e for A2A query
    std::map<int, input_struct> input_file41{
        {0, input_struct("RM_10092", "RM_50_poi_on_10092.txt", 0, 1)},
    };

    for (int i = 0; i < input_file41.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file41, output_folder_list[40], e, input_file_index, 5);
        }
    }

    // run BH dataset on large POI when varying e for ablation study for A2A query
    std::map<int, input_struct> input_file42{
        {0, input_struct("BH_500835", "BH_500_poi_on_500835.txt", 0, 1)},
    };

    for (int i = 0; i < input_file42.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file42, output_folder_list[41], e, input_file_index, 6);
        }
    }

    // run EP dataset on large POI when varying e for ablation study for A2A query
    std::map<int, input_struct> input_file43{
        {0, input_struct("EP_500384", "EP_500_poi_on_500384.txt", 0, 1)},
    };

    for (int i = 0; i < input_file43.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file43, output_folder_list[42], e, input_file_index, 6);
        }
    }

    // run GF dataset on large POI when varying e for ablation study for A2A query
    std::map<int, input_struct> input_file44{
        {0, input_struct("GF_500208", "GF_500_poi_on_500208.txt", 0, 1)},
    };

    for (int i = 0; i < input_file44.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file44, output_folder_list[43], e, input_file_index, 6);
        }
    }

    // run LM dataset on large POI when varying e for ablation study for A2A query
    std::map<int, input_struct> input_file45{
        {0, input_struct("LM_500208", "LM_500_poi_on_500208.txt", 0, 1)},
    };

    for (int i = 0; i < input_file45.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file45, output_folder_list[44], e, input_file_index, 6);
        }
    }

    // run RM dataset on large POI when varying e for ablation study for A2A query
    std::map<int, input_struct> input_file46{
        {0, input_struct("RM_500208", "RM_500_poi_on_500208.txt", 0, 1)},
    };

    for (int i = 0; i < input_file46.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file46, output_folder_list[45], e, input_file_index, 6);
        }
    }

    // run BH small dataset on small POI when varying e for proximity query oracle and algorithm comparision for A2A query
    std::map<int, input_struct> input_file47{
        {0, input_struct("BH_10086", "BH_50_poi_on_10086.txt", 0, 1)},
    };

    for (int i = 0; i < input_file47.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file47, output_folder_list[46], e, input_file_index, 7);
        }
    }

    // run EP small dataset on small POI when varying e for proximity query oracle and algorithm comparision for A2A query
    std::map<int, input_struct> input_file48{
        {0, input_struct("EP_10062", "EP_50_poi_on_10062.txt", 0, 1)},
    };

    for (int i = 0; i < input_file48.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file48, output_folder_list[47], e, input_file_index, 7);
        }
    }

    // run GF small dataset on small POI when varying e for proximity query oracle and algorithm comparision for A2A query
    std::map<int, input_struct> input_file49{
        {0, input_struct("GF_10092", "GF_50_poi_on_10092.txt", 0, 1)},
    };

    for (int i = 0; i < input_file49.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file49, output_folder_list[48], e, input_file_index, 7);
        }
    }

    // run LM small dataset on small POI when varying e for proximity query oracle and algorithm comparision for A2A query
    std::map<int, input_struct> input_file50{
        {0, input_struct("LM_10092", "LM_50_poi_on_10092.txt", 0, 1)},
    };

    for (int i = 0; i < input_file50.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file50, output_folder_list[49], e, input_file_index, 7);
        }
    }

    // run RM small dataset on small POI when varying e for proximity query oracle and algorithm comparision for A2A query
    std::map<int, input_struct> input_file51{
        {0, input_struct("RM_10092", "RM_50_poi_on_10092.txt", 0, 1)},
    };

    for (int i = 0; i < input_file51.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file51, output_folder_list[50], e, input_file_index, 7);
        }
    }

    // run BH small dataset on small POI when varying e on TIN dataset
    std::map<int, input_struct> input_file52{
        {0, input_struct("BH_10086", "BH_50_poi_on_10086.txt", 0, 1)},
    };
    for (int i = 0; i < input_file52.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file52, output_folder_list[51], e, input_file_index, 8);
        }
    }

    // run BH small dataset on small POI when varying POI number on TIN dataset
    std::map<int, input_struct> input_file53{
        {0, input_struct("BH_10086", "BH_50_poi_on_10086.txt", 0, 1)},
        {1, input_struct("BH_10086", "BH_100_poi_on_10086.txt", 0, 1)},
        {2, input_struct("BH_10086", "BH_150_poi_on_10086.txt", 0, 1)},
        {3, input_struct("BH_10086", "BH_200_poi_on_10086.txt", 0, 1)},
        {4, input_struct("BH_10086", "BH_250_poi_on_10086.txt", 0, 1)},
    };
    for (int i = 0; i < input_file53.size(); i++)
    {
        double e = 0.1;
        int input_file_index = i;

        run_algorithms(input_file53, output_folder_list[52], e, input_file_index, 8);
    }

    // run EP small dataset on small POI when varying e on TIN dataset
    std::map<int, input_struct> input_file54{
        {0, input_struct("EP_10062", "EP_50_poi_on_10062.txt", 0, 1)},
    };

    for (int i = 0; i < input_file54.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file54, output_folder_list[53], e, input_file_index, 8);
        }
    }

    // run EP small dataset on small POI when varying POI number on TIN dataset
    std::map<int, input_struct> input_file55{
        {0, input_struct("EP_10062", "EP_50_poi_on_10062.txt", 0, 1)},
        {1, input_struct("EP_10062", "EP_100_poi_on_10062.txt", 0, 1)},
        {2, input_struct("EP_10062", "EP_150_poi_on_10062.txt", 0, 1)},
        {3, input_struct("EP_10062", "EP_200_poi_on_10062.txt", 0, 1)},
        {4, input_struct("EP_10062", "EP_250_poi_on_10062.txt", 0, 1)},
    };
    for (int i = 0; i < input_file55.size(); i++)
    {
        double e = 0.1;
        int input_file_index = i;

        run_algorithms(input_file55, output_folder_list[54], e, input_file_index, 8);
    }

    // run EP small dataset on small POI when varying dataset size on TIN dataset
    std::map<int, input_struct> input_file56{
        {0, input_struct("EP_10062", "EP_50_poi_on_10062.txt", 0, 1)},
        {1, input_struct("EP_20130", "EP_50_poi_on_20130.txt", 0, 1)},
        {2, input_struct("EP_30098", "EP_50_poi_on_30098.txt", 0, 1)},
        {3, input_struct("EP_40076", "EP_50_poi_on_40076.txt", 0, 1)},
        {4, input_struct("EP_50373", "EP_50_poi_on_50373.txt", 0, 1)},
    };
    for (int i = 0; i < input_file56.size(); i++)
    {
        double e = 0.1;
        int input_file_index = i;

        run_algorithms(input_file56, output_folder_list[55], e, input_file_index, 8);
    }

    // run GF small dataset on small POI when varying e on TIN dataset
    std::map<int, input_struct> input_file57{
        {0, input_struct("GF_10092", "GF_50_poi_on_10092.txt", 0, 1)},
    };

    for (int i = 0; i < input_file57.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file57, output_folder_list[56], e, input_file_index, 8);
        }
    }

    // run GF small dataset on small POI when varying POI number on TIN dataset
    std::map<int, input_struct> input_file58{
        {0, input_struct("GF_10092", "GF_50_poi_on_10092.txt", 0, 1)},
        {1, input_struct("GF_10092", "GF_100_poi_on_10092.txt", 0, 1)},
        {2, input_struct("GF_10092", "GF_150_poi_on_10092.txt", 0, 1)},
        {3, input_struct("GF_10092", "GF_200_poi_on_10092.txt", 0, 1)},
        {4, input_struct("GF_10092", "GF_250_poi_on_10092.txt", 0, 1)},
    };
    for (int i = 0; i < input_file58.size(); i++)
    {
        double e = 0.1;
        int input_file_index = i;

        run_algorithms(input_file58, output_folder_list[57], e, input_file_index, 8);
    }

    // run LM small dataset on small POI when varying e on TIN dataset
    std::map<int, input_struct> input_file59{
        {0, input_struct("LM_10092", "LM_50_poi_on_10092.txt", 0, 1)},
    };

    for (int i = 0; i < input_file59.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file59, output_folder_list[58], e, input_file_index, 8);
        }
    }

    // run LM small dataset on small POI when varying POI number on TIN dataset
    std::map<int, input_struct> input_file60{
        {0, input_struct("LM_10092", "LM_50_poi_on_10092.txt", 0, 1)},
        {1, input_struct("LM_10092", "LM_100_poi_on_10092.txt", 0, 1)},
        {2, input_struct("LM_10092", "LM_150_poi_on_10092.txt", 0, 1)},
        {3, input_struct("LM_10092", "LM_200_poi_on_10092.txt", 0, 1)},
        {4, input_struct("LM_10092", "LM_250_poi_on_10092.txt", 0, 1)},
    };
    for (int i = 0; i < input_file60.size(); i++)
    {
        double e = 0.1;
        int input_file_index = i;

        run_algorithms(input_file60, output_folder_list[59], e, input_file_index, 8);
    }

    // run RM small dataset on small POI when varying e on TIN dataset
    std::map<int, input_struct> input_file61{
        {0, input_struct("RM_10092", "RM_50_poi_on_10092.txt", 0, 1)},
    };

    for (int i = 0; i < input_file61.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file61, output_folder_list[60], e, input_file_index, 8);
        }
    }

    // run RM small dataset on small POI when varying POI number on TIN dataset
    std::map<int, input_struct> input_file62{
        {0, input_struct("RM_10092", "RM_50_poi_on_10092.txt", 0, 1)},
        {1, input_struct("RM_10092", "RM_100_poi_on_10092.txt", 0, 1)},
        {2, input_struct("RM_10092", "RM_150_poi_on_10092.txt", 0, 1)},
        {3, input_struct("RM_10092", "RM_200_poi_on_10092.txt", 0, 1)},
        {4, input_struct("RM_10092", "RM_250_poi_on_10092.txt", 0, 1)},
    };
    for (int i = 0; i < input_file62.size(); i++)
    {
        double e = 0.1;
        int input_file_index = i;

        run_algorithms(input_file62, output_folder_list[61], e, input_file_index, 8);
    }

    // run BH dataset on large POI when varying e on TIN dataset
    std::map<int, input_struct> input_file63{
        {0, input_struct("BH_500835", "BH_500_poi_on_500835.txt", 0, 1)},
    };

    for (int i = 0; i < input_file63.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file63, output_folder_list[62], e, input_file_index, 9);
        }
    }

    // run BH dataset on large POI when varying POI number on TIN dataset
    std::map<int, input_struct> input_file64{
        {0, input_struct("BH_500835", "BH_500_poi_on_500835.txt", 0, 1)},
        {1, input_struct("BH_500835", "BH_1000_poi_on_500835.txt", 0, 1)},
        {2, input_struct("BH_500835", "BH_1500_poi_on_500835.txt", 0, 1)},
        {3, input_struct("BH_500835", "BH_2000_poi_on_500835.txt", 0, 1)},
        {4, input_struct("BH_500835", "BH_2500_poi_on_500835.txt", 0, 1)},
    };
    for (int i = 0; i < input_file64.size(); i++)
    {
        double e = 0.25;
        int input_file_index = i;

        run_algorithms(input_file64, output_folder_list[63], e, input_file_index, 9);
    }

    // run BH dataset on large POI when varying dataset size on TIN dataset
    std::map<int, input_struct> input_file65{
        {0, input_struct("BH_500835", "BH_500_poi_on_500835.txt", 0, 1)},
        {1, input_struct("BH_1000414", "BH_500_poi_on_1000414.txt", 0, 1)},
        {2, input_struct("BH_1500996", "BH_500_poi_on_1500996.txt", 0, 1)},
        {3, input_struct("BH_2001610", "BH_500_poi_on_2001610.txt", 0, 1)},
        {4, input_struct("BH_2502596", "BH_500_poi_on_2502596.txt", 0, 1)},
    };
    for (int i = 0; i < input_file65.size(); i++)
    {
        double e = 0.25;
        int input_file_index = i;

        run_algorithms(input_file65, output_folder_list[64], e, input_file_index, 9);
    }

    // run EP dataset on large POI when varying e on TIN dataset
    std::map<int, input_struct> input_file66{
        {0, input_struct("EP_500384", "EP_500_poi_on_500384.txt", 0, 1)},
    };

    for (int i = 0; i < input_file66.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file66, output_folder_list[65], e, input_file_index, 9);
        }
    }

    // run EP dataset on large POI when varying POI number on TIN dataset
    std::map<int, input_struct> input_file67{
        {0, input_struct("EP_500384", "EP_500_poi_on_500384.txt", 0, 1)},
        {1, input_struct("EP_500384", "EP_1000_poi_on_500384.txt", 0, 1)},
        {2, input_struct("EP_500384", "EP_1500_poi_on_500384.txt", 0, 1)},
        {3, input_struct("EP_500384", "EP_2000_poi_on_500384.txt", 0, 1)},
        {4, input_struct("EP_500384", "EP_2500_poi_on_500384.txt", 0, 1)},
    };
    for (int i = 0; i < input_file67.size(); i++)
    {
        double e = 0.25;
        int input_file_index = i;

        run_algorithms(input_file67, output_folder_list[66], e, input_file_index, 9);
    }

    // run EP dataset on large POI when varying dataset size on TIN dataset
    std::map<int, input_struct> input_file68{
        {0, input_struct("EP_500384", "EP_500_poi_on_500384.txt", 0, 1)},
        {1, input_struct("EP_1001040", "EP_500_poi_on_1001040.txt", 0, 1)},
        {2, input_struct("EP_1501578", "EP_500_poi_on_1501578.txt", 0, 1)},
        {3, input_struct("EP_2001536", "EP_500_poi_on_2001536.txt", 0, 1)},
        {4, input_struct("EP_2500560", "EP_500_poi_on_2500560.txt", 0, 1)},
    };
    for (int i = 0; i < input_file68.size(); i++)
    {
        double e = 0.25;
        int input_file_index = i;

        run_algorithms(input_file68, output_folder_list[67], e, input_file_index, 9);
    }

    // run GF dataset on large POI when varying e on TIN dataset
    std::map<int, input_struct> input_file69{
        {0, input_struct("GF_500208", "GF_500_poi_on_500208.txt", 0, 1)},
    };

    for (int i = 0; i < input_file69.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file69, output_folder_list[68], e, input_file_index, 9);
        }
    }

    // run GF dataset on large POI when varying POI number on TIN dataset
    std::map<int, input_struct> input_file70{
        {0, input_struct("GF_500208", "GF_500_poi_on_500208.txt", 0, 1)},
        {1, input_struct("GF_500208", "GF_1000_poi_on_500208.txt", 0, 1)},
        {2, input_struct("GF_500208", "GF_1500_poi_on_500208.txt", 0, 1)},
        {3, input_struct("GF_500208", "GF_2000_poi_on_500208.txt", 0, 1)},
        {4, input_struct("GF_500208", "GF_2500_poi_on_500208.txt", 0, 1)},
    };
    for (int i = 0; i < input_file70.size(); i++)
    {
        double e = 0.25;
        int input_file_index = i;

        run_algorithms(input_file70, output_folder_list[69], e, input_file_index, 9);
    }

    // run GF dataset on large POI when varying dataset size on TIN dataset
    std::map<int, input_struct> input_file71{
        {0, input_struct("GF_500208", "GF_500_poi_on_500208.txt", 0, 1)},
        {1, input_struct("GF_1000518", "GF_500_poi_on_1000518.txt", 0, 1)},
        {2, input_struct("GF_1501668", "GF_500_poi_on_1501668.txt", 0, 1)},
        {3, input_struct("GF_2000832", "GF_500_poi_on_2000832.txt", 0, 1)},
        {4, input_struct("GF_2502075", "GF_500_poi_on_2502075.txt", 0, 1)},
    };
    for (int i = 0; i < input_file71.size(); i++)
    {
        double e = 0.25;
        int input_file_index = i;

        run_algorithms(input_file71, output_folder_list[70], e, input_file_index, 9);
    }

    // run LM dataset on large POI when varying e on TIN dataset
    std::map<int, input_struct> input_file72{
        {0, input_struct("LM_500208", "LM_500_poi_on_500208.txt", 0, 1)},
    };

    for (int i = 0; i < input_file72.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file72, output_folder_list[71], e, input_file_index, 9);
        }
    }

    // run LM dataset on large POI when varying POI number on TIN dataset
    std::map<int, input_struct> input_file73{
        {0, input_struct("LM_500208", "LM_500_poi_on_500208.txt", 0, 1)},
        {1, input_struct("LM_500208", "LM_1000_poi_on_500208.txt", 0, 1)},
        {2, input_struct("LM_500208", "LM_1500_poi_on_500208.txt", 0, 1)},
        {3, input_struct("LM_500208", "LM_2000_poi_on_500208.txt", 0, 1)},
        {4, input_struct("LM_500208", "LM_2500_poi_on_500208.txt", 0, 1)},
    };
    for (int i = 0; i < input_file73.size(); i++)
    {
        double e = 0.25;
        int input_file_index = i;

        run_algorithms(input_file73, output_folder_list[72], e, input_file_index, 9);
    }

    // run LM dataset on large POI when varying dataset size on TIN dataset
    std::map<int, input_struct> input_file74{
        {0, input_struct("LM_500208", "LM_500_poi_on_500208.txt", 0, 1)},
        {1, input_struct("LM_1000518", "LM_500_poi_on_1000518.txt", 0, 1)},
        {2, input_struct("LM_1501668", "LM_500_poi_on_1501668.txt", 0, 1)},
        {3, input_struct("LM_2000832", "LM_500_poi_on_2000832.txt", 0, 1)},
        {4, input_struct("LM_2502075", "LM_500_poi_on_2502075.txt", 0, 1)},
    };
    for (int i = 0; i < input_file74.size(); i++)
    {
        double e = 0.25;
        int input_file_index = i;

        run_algorithms(input_file74, output_folder_list[73], e, input_file_index, 9);
    }

    // run RM dataset on large POI when varying e on TIN dataset
    std::map<int, input_struct> input_file75{
        {0, input_struct("RM_500208", "RM_500_poi_on_500208.txt", 0, 1)},
    };

    for (int i = 0; i < input_file75.size(); i++)
    {
        for (int j = 0; j < e_list.size(); j++)
        {
            double e = e_list[j];
            int input_file_index = i;

            run_algorithms(input_file75, output_folder_list[74], e, input_file_index, 9);
        }
    }

    // run RM dataset on large POI when varying POI number on TIN dataset
    std::map<int, input_struct> input_file76{
        {0, input_struct("RM_500208", "RM_500_poi_on_500208.txt", 0, 1)},
        {1, input_struct("RM_500208", "RM_1000_poi_on_500208.txt", 0, 1)},
        {2, input_struct("RM_500208", "RM_1500_poi_on_500208.txt", 0, 1)},
        {3, input_struct("RM_500208", "RM_2000_poi_on_500208.txt", 0, 1)},
        {4, input_struct("RM_500208", "RM_2500_poi_on_500208.txt", 0, 1)},
    };
    for (int i = 0; i < input_file76.size(); i++)
    {
        double e = 0.25;
        int input_file_index = i;

        run_algorithms(input_file76, output_folder_list[75], e, input_file_index, 9);
    }

    // run RM dataset on large POI when varying dataset size on TIN dataset
    std::map<int, input_struct> input_file77{
        {0, input_struct("RM_500208", "RM_500_poi_on_500208.txt", 0, 1)},
        {1, input_struct("RM_1000518", "RM_500_poi_on_1000518.txt", 0, 1)},
        {2, input_struct("RM_1501668", "RM_500_poi_on_1501668.txt", 0, 1)},
        {3, input_struct("RM_2000832", "RM_500_poi_on_2000832.txt", 0, 1)},
        {4, input_struct("RM_2502075", "RM_500_poi_on_2502075.txt", 0, 1)},
    };
    for (int i = 0; i < input_file77.size(); i++)
    {
        double e = 0.25;
        int input_file_index = i;

        run_algorithms(input_file77, output_folder_list[76], e, input_file_index, 9);
    }
}
