#include "helper.h"
#include <sstream>
#include <unistd.h>
#include <sys/resource.h>
#include <sys/times.h>
#include <iostream>
#include <sstream>
#include <chrono>
#include <unordered_map>
#include <set>

void point_cloud_to_terrain_and_initialize_terrain(
    point_cloud_geodesic::PointCloud *point_cloud, geodesic::Mesh *mesh,
    double &point_cloud_to_terrain_time, double &point_cloud_to_terrain_memory_usage)
{
    auto start_point_cloud_to_terrain_time = std::chrono::high_resolution_clock::now();

    std::string terrain_write_path = "temp_terrain.off";
    point_cloud->point_cloud_to_terrain(point_cloud_to_terrain_memory_usage);
    std::vector<double> terrain_points;
    std::vector<unsigned> terrain_faces;
    geodesic::read_mesh_from_file(&terrain_write_path[0], terrain_points, terrain_faces);
    mesh->initialize_mesh_data(terrain_points, terrain_faces);
    remove(&terrain_write_path[0]);

    auto stop_point_cloud_to_terrain_time = std::chrono::high_resolution_clock::now();
    auto duration_point_cloud_to_terrain_time = std::chrono::duration_cast<std::chrono::microseconds>(stop_point_cloud_to_terrain_time - start_point_cloud_to_terrain_time);
    point_cloud_to_terrain_time = duration_point_cloud_to_terrain_time.count();
    point_cloud_to_terrain_time /= 1000;
}

void calculate_point_cloud_exact_distance(
    point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list,
    int source_poi_index, int destination_poi_index, double &point_cloud_exact_distance)
{
    point_cloud_geodesic::PointCloudGeodesicAlgorithmDijkstra algorithm(point_cloud);
    double const distance_limit = point_cloud_geodesic::INFIN;
    point_cloud_geodesic::PathPoint source(&point_cloud->pc_points()[poi_list[source_poi_index]]);
    point_cloud_geodesic::PathPoint destination(&point_cloud->pc_points()[poi_list[destination_poi_index]]);
    std::vector<point_cloud_geodesic::PathPoint> one_source_poi_list(1, source);
    std::vector<point_cloud_geodesic::PathPoint> one_destination_poi_list(1, destination);
    algorithm.propagate(one_source_poi_list, distance_limit);
    algorithm.best_source(destination, point_cloud_exact_distance);
    point_cloud_exact_distance = round(point_cloud_exact_distance * 1000000000.0) / 1000000000.0;
}

void calculate_terrain_exact_distance(
    point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list,
    int source_poi_index, int destination_poi_index, double &terrain_exact_distance)
{
    double point_cloud_to_terrain_time = 0;
    double point_cloud_to_terrain_memory_usage = 0;
    geodesic::Mesh mesh;
    point_cloud_to_terrain_and_initialize_terrain(point_cloud, &mesh, point_cloud_to_terrain_time, point_cloud_to_terrain_memory_usage);

    geodesic::GeodesicAlgorithmExact algorithm(&mesh);
    double const distance_limit = geodesic::GEODESIC_INF;
    geodesic::SurfacePoint source(&mesh.vertices()[poi_list[source_poi_index]]);
    geodesic::SurfacePoint destination(&mesh.vertices()[poi_list[destination_poi_index]]);
    std::vector<geodesic::SurfacePoint> one_source_poi_list(1, source);
    std::vector<geodesic::SurfacePoint> one_destination_poi_list(1, destination);
    algorithm.propagate(one_source_poi_list, distance_limit);
    algorithm.best_source(destination, terrain_exact_distance);
    terrain_exact_distance = round(terrain_exact_distance * 1000000000.0) / 1000000000.0;
}

void calculate_point_cloud_exact_all_poi_knn_or_range_query(
    point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list, int knn_one_range_two,
    int k_value, double range, std::vector<std::vector<int>> &all_poi_knn_or_range_list)
{
    std::vector<std::vector<std::pair<double, int>>> poi_to_other_poi_distance_and_index_list;
    poi_to_other_poi_distance_and_index_list.clear();
    std::vector<point_cloud_geodesic::PathPoint> one_source_poi_list;
    std::vector<std::pair<double, int>> one_poi_to_other_poi_distance_and_index_list;
    point_cloud_geodesic::PointCloudGeodesicAlgorithmDijkstra algorithm(point_cloud);
    double const distance_limit = point_cloud_geodesic::INFIN;
    for (int i = 0; i < poi_list.size(); i++)
    {
        one_source_poi_list.clear();
        one_poi_to_other_poi_distance_and_index_list.clear();
        one_source_poi_list.push_back(point_cloud_geodesic::PathPoint(&point_cloud->pc_points()[poi_list[i]]));
        algorithm.propagate(one_source_poi_list, distance_limit);
        for (int j = 0; j < poi_list.size(); j++)
        {
            double distance;
            point_cloud_geodesic::PathPoint one_destination_poi(&point_cloud->pc_points()[poi_list[j]]);
            algorithm.best_source(one_destination_poi, distance);
            one_poi_to_other_poi_distance_and_index_list.push_back(std::make_pair(distance, j));
        }
        std::sort(one_poi_to_other_poi_distance_and_index_list.begin(), one_poi_to_other_poi_distance_and_index_list.end());
        poi_to_other_poi_distance_and_index_list.push_back(one_poi_to_other_poi_distance_and_index_list);
    }
    knn_or_range_query(knn_one_range_two, k_value, range, poi_to_other_poi_distance_and_index_list, all_poi_knn_or_range_list);
}

void calculate_terrain_exact_all_poi_knn_or_range_query(
    point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list, int knn_one_range_two,
    int k_value, double range, std::vector<std::vector<int>> &all_poi_knn_or_range_list)
{
    double point_cloud_to_terrain_time = 0;
    double point_cloud_to_terrain_memory_usage = 0;
    geodesic::Mesh mesh;
    point_cloud_to_terrain_and_initialize_terrain(point_cloud, &mesh, point_cloud_to_terrain_time, point_cloud_to_terrain_memory_usage);

    std::vector<std::vector<std::pair<double, int>>> poi_to_other_poi_distance_and_index_list;
    poi_to_other_poi_distance_and_index_list.clear();
    std::vector<geodesic::SurfacePoint> one_source_poi_list;
    std::vector<std::pair<double, int>> one_poi_to_other_poi_distance_and_index_list;
    geodesic::GeodesicAlgorithmExact algorithm(&mesh);
    double const distance_limit = geodesic::GEODESIC_INF;
    for (int i = 0; i < poi_list.size(); i++)
    {
        one_source_poi_list.clear();
        one_poi_to_other_poi_distance_and_index_list.clear();
        one_source_poi_list.push_back(geodesic::SurfacePoint(&mesh.vertices()[poi_list[i]]));
        algorithm.propagate(one_source_poi_list, distance_limit);
        for (int j = 0; j < poi_list.size(); j++)
        {
            double distance;
            geodesic::SurfacePoint one_destination_poi(&mesh.vertices()[poi_list[j]]);
            algorithm.best_source(one_destination_poi, distance);
            one_poi_to_other_poi_distance_and_index_list.push_back(std::make_pair(distance, j));
        }
        std::sort(one_poi_to_other_poi_distance_and_index_list.begin(), one_poi_to_other_poi_distance_and_index_list.end());
        poi_to_other_poi_distance_and_index_list.push_back(one_poi_to_other_poi_distance_and_index_list);
    }
    knn_or_range_query(knn_one_range_two, k_value, range, poi_to_other_poi_distance_and_index_list, all_poi_knn_or_range_list);
}

void RC_Oracle(int poi_num, point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list, double epsilon,
               std::unordered_map<int, double> &distance_poi_to_poi_map,
               std::unordered_map<int, std::vector<point_cloud_geodesic::PathPoint>> &path_poi_to_poi_map,
               std::unordered_map<int, int> &non_exact_source_poi_map,
               std::unordered_map<int, int> &exact_source_poi_process_order_map,
               double &construction_time, double &memory_usage, double &index_size)
{
    auto start_construction_time = std::chrono::high_resolution_clock::now();

    point_cloud_geodesic::PointCloudGeodesicAlgorithmDijkstra algorithm(point_cloud);
    double const distance_limit = point_cloud_geodesic::INFIN;
    distance_poi_to_poi_map.clear();
    path_poi_to_poi_map.clear();
    non_exact_source_poi_map.clear();
    int index_path_size = 0;
    int index_path_num = 0;
    std::vector<point_cloud_geodesic::PathPoint> parent_one_source_poi_list;
    std::vector<point_cloud_geodesic::PathPoint> parent_destinations_poi_list;
    std::vector<double> poi_x_or_y_coordinate_list(poi_num, 0);
    std::vector<std::pair<double, int>> sorted_poi_x_or_y_coordinate_and_original_index_list;
    sorted_poi_x_or_y_coordinate_and_original_index_list.clear();
    double max_increment = 0;

    // sort poi based on their x coordinate, since the point cloud in x-axis is larger
    if (point_cloud->m_xlength > point_cloud->m_ylength)
    {
        for (int i = 0; i < poi_num; i++)
        {
            poi_x_or_y_coordinate_list[i] = point_cloud->pc_points()[poi_list[i]].getx();
        }
        max_increment = point_cloud->m_xincrement;
    }
    // sort poi based on their y coordinate, since the point cloud in y-axis is larger
    else
    {
        for (int i = 0; i < poi_num; i++)
        {
            poi_x_or_y_coordinate_list[i] = point_cloud->pc_points()[poi_list[i]].gety();
        }
        max_increment = point_cloud->m_yincrement;
    }
    sort_min_to_max_and_get_original_index(poi_x_or_y_coordinate_list, sorted_poi_x_or_y_coordinate_and_original_index_list);
    assert(sorted_poi_x_or_y_coordinate_and_original_index_list.size() == poi_num);

    std::vector<int> processed_poi_index_list;
    std::unordered_map<int, int> processed_poi_index_unordered_map;
    processed_poi_index_list.clear();
    processed_poi_index_unordered_map.clear();
    int exact_source_poi_process_order = 1;

    // for each non-processes poi, select the poi with smallest x/y coordinate, mark it as
    // parent source poi, run SSAD from this poi
    for (int i = 0; i < poi_num; i++)
    {
        parent_one_source_poi_list.clear();
        parent_destinations_poi_list.clear();

        std::vector<double> distance_of_current_parent_poi_to_non_precessed_poi_list(poi_num, 0);
        std::vector<std::pair<double, int>> sorted_distance_of_current_parent_poi_to_non_precessed_poi_and_original_index_list;
        sorted_distance_of_current_parent_poi_to_non_precessed_poi_and_original_index_list.clear();

        int current_parent_source_poi_index = sorted_poi_x_or_y_coordinate_and_original_index_list[i].second;
        if (processed_poi_index_unordered_map.count(current_parent_source_poi_index) == 0)
        {
            exact_source_poi_process_order_map[current_parent_source_poi_index] = exact_source_poi_process_order;
            exact_source_poi_process_order++;

            processed_poi_index_unordered_map[current_parent_source_poi_index] = current_parent_source_poi_index;
            parent_one_source_poi_list.push_back(point_cloud_geodesic::PathPoint(&point_cloud->pc_points()[poi_list[current_parent_source_poi_index]]));

            for (int j = 0; j < poi_num; j++)
            {
                int other_parent_destination_poi_index = sorted_poi_x_or_y_coordinate_and_original_index_list[j].second;
                if (processed_poi_index_unordered_map.count(other_parent_destination_poi_index) == 0)
                {
                    parent_destinations_poi_list.push_back(point_cloud_geodesic::PathPoint(&point_cloud->pc_points()[poi_list[other_parent_destination_poi_index]]));
                }
            }

            algorithm.propagate(parent_one_source_poi_list, &parent_destinations_poi_list);

            // store the distance and path to these parent destination pois
            for (int j = 0; j < poi_num; j++)
            {
                int other_parent_destination_poi_index = sorted_poi_x_or_y_coordinate_and_original_index_list[j].second;
                if (processed_poi_index_unordered_map.count(other_parent_destination_poi_index) == 0)
                {
                    std::vector<point_cloud_geodesic::PathPoint> path;
                    point_cloud_geodesic::PathPoint one_destination(&point_cloud->pc_points()[poi_list[other_parent_destination_poi_index]]);
                    algorithm.trace_back(one_destination, path);

                    int parent_src_parent_dest_index;
                    if (current_parent_source_poi_index <= other_parent_destination_poi_index)
                    {
                        hash_function_two_keys_to_one_key(poi_num, current_parent_source_poi_index, other_parent_destination_poi_index, parent_src_parent_dest_index);
                    }
                    else
                    {
                        hash_function_two_keys_to_one_key(poi_num, other_parent_destination_poi_index, current_parent_source_poi_index, parent_src_parent_dest_index);
                        std::reverse(path.begin(), path.end());
                    }
                    distance_poi_to_poi_map[parent_src_parent_dest_index] = length(path);
                    path_poi_to_poi_map[parent_src_parent_dest_index] = path;
                    index_path_size += path.size();
                    index_path_num++;
                    distance_of_current_parent_poi_to_non_precessed_poi_list[other_parent_destination_poi_index] = length(path);
                }
            }

            sort_min_to_max_and_get_original_index(distance_of_current_parent_poi_to_non_precessed_poi_list, sorted_distance_of_current_parent_poi_to_non_precessed_poi_and_original_index_list);
            assert(sorted_distance_of_current_parent_poi_to_non_precessed_poi_and_original_index_list.size() == poi_num);

            // for these parent destination pois, check whether they are close to parent source poi, if so,
            // we need to (1) use the path between parent source poi and a children destination poi that are
            // far away to approximate the path between children source poi and this far children destination
            // poi, or (2) run SSAD from children source poi to children destination poi that are near =>
            // based on the euclidean distance between children source poi and a far/near children destination poi
            std::vector<point_cloud_geodesic::PathPoint> children_one_source_poi_list;
            std::vector<point_cloud_geodesic::PathPoint> children_destinations_poi_list;
            std::vector<int> children_destinations_poi_index_list;

            for (int j = 0; j < poi_num; j++)
            {
                if (sorted_distance_of_current_parent_poi_to_non_precessed_poi_and_original_index_list[j].first == 0)
                {
                    continue;
                }
                if (sorted_distance_of_current_parent_poi_to_non_precessed_poi_and_original_index_list[j].first <= max_increment * 7)
                {
                    children_one_source_poi_list.clear();
                    children_destinations_poi_list.clear();
                    children_destinations_poi_index_list.clear();

                    int current_children_source_poi_index = sorted_distance_of_current_parent_poi_to_non_precessed_poi_and_original_index_list[j].second;
                    if (processed_poi_index_unordered_map.count(current_children_source_poi_index) == 0)
                    {
                        processed_poi_index_unordered_map[current_children_source_poi_index] = current_children_source_poi_index;
                        children_one_source_poi_list.push_back(point_cloud_geodesic::PathPoint(&point_cloud->pc_points()[poi_list[current_children_source_poi_index]]));

                        assert(non_exact_source_poi_map.count(current_children_source_poi_index) == 0);
                        non_exact_source_poi_map[current_children_source_poi_index] = current_parent_source_poi_index;

                        // the shorter part of the path between children source poi and parent source poi
                        int children_src_parent_src_index;
                        double children_src_parent_src_distance = 0;
                        std::vector<point_cloud_geodesic::PathPoint> children_src_parent_src_path;
                        children_src_parent_src_path.clear();
                        bool reverse_path_children_src_parent_src = false;

                        if (current_children_source_poi_index <= current_parent_source_poi_index)
                        {
                            hash_function_two_keys_to_one_key(poi_num, current_children_source_poi_index, current_parent_source_poi_index, children_src_parent_src_index);
                        }
                        else
                        {
                            hash_function_two_keys_to_one_key(poi_num, current_parent_source_poi_index, current_children_source_poi_index, children_src_parent_src_index);
                            reverse_path_children_src_parent_src = true;
                        }
                        children_src_parent_src_distance = distance_poi_to_poi_map[children_src_parent_src_index];
                        children_src_parent_src_path = path_poi_to_poi_map[children_src_parent_src_index];
                        if (reverse_path_children_src_parent_src)
                        {
                            std::reverse(children_src_parent_src_path.begin(), children_src_parent_src_path.end());
                        }

                        // for each children destination poi, (1) if the euclidean distance between current children
                        // source poi and this children destination poi is less than a value, then run SSAD to this
                        // children destination poi, (2) if not, just use the existing path between current partent
                        // source poi to this children destination poi to approximate the path between current
                        // children source poi and this children destination poi
                        for (int k = 0; k < poi_num; k++)
                        {
                            int other_children_destination_poi_index = sorted_distance_of_current_parent_poi_to_non_precessed_poi_and_original_index_list[k].second;

                            if (processed_poi_index_unordered_map.count(other_children_destination_poi_index) == 0)
                            {

                                double current_children_source_poi_x = point_cloud->pc_points()[poi_list[current_children_source_poi_index]].getx();
                                double current_children_source_poi_y = point_cloud->pc_points()[poi_list[current_children_source_poi_index]].gety();
                                double other_children_destination_poi_x = point_cloud->pc_points()[poi_list[other_children_destination_poi_index]].getx();
                                double other_children_destination_poi_y = point_cloud->pc_points()[poi_list[other_children_destination_poi_index]].gety();
                                double euclidean_distance_curr_children_other_children = euclidean_distance(current_children_source_poi_x, current_children_source_poi_y, other_children_destination_poi_x, other_children_destination_poi_y);

                                // no need run SSAD to this children destination poi since it is far away from current children source poi
                                if (euclidean_distance_curr_children_other_children > 2 * sorted_distance_of_current_parent_poi_to_non_precessed_poi_and_original_index_list[j].first / pow(epsilon, 0.25))
                                {
                                    // if other children distination poi has same parent source with the current children source poi
                                    if (sorted_distance_of_current_parent_poi_to_non_precessed_poi_and_original_index_list[k].first <= max_increment * 7)
                                    {
                                        // the longer part of the path between parent source poi and children destination poi
                                        int parent_src_children_dest_index;
                                        double parent_src_children_dest_distance = 0;
                                        std::vector<point_cloud_geodesic::PathPoint> parent_src_children_dest_path;
                                        parent_src_children_dest_path.clear();
                                        bool reverse_path_parent_src_children_dest = false;

                                        if (current_parent_source_poi_index <= other_children_destination_poi_index)
                                        {
                                            hash_function_two_keys_to_one_key(poi_num, current_parent_source_poi_index, other_children_destination_poi_index, parent_src_children_dest_index);
                                        }
                                        else
                                        {
                                            hash_function_two_keys_to_one_key(poi_num, other_children_destination_poi_index, current_parent_source_poi_index, parent_src_children_dest_index);
                                            reverse_path_parent_src_children_dest = true;
                                        }
                                        parent_src_children_dest_distance = distance_poi_to_poi_map[parent_src_children_dest_index];
                                        parent_src_children_dest_path = path_poi_to_poi_map[parent_src_children_dest_index];
                                        if (reverse_path_parent_src_children_dest)
                                        {
                                            std::reverse(parent_src_children_dest_path.begin(), parent_src_children_dest_path.end());
                                        }

                                        // sum the shorter and longer path together to be the new approximated path
                                        int children_src_children_dest_index;
                                        double children_src_children_dest_distance = 0;
                                        std::vector<point_cloud_geodesic::PathPoint> children_src_children_dest_path;
                                        children_src_children_dest_path.clear();
                                        bool reverse_path_children_src_children_dest = false;
                                        if (current_children_source_poi_index <= other_children_destination_poi_index)
                                        {
                                            hash_function_two_keys_to_one_key(poi_num, current_children_source_poi_index, other_children_destination_poi_index, children_src_children_dest_index);
                                        }
                                        else
                                        {
                                            hash_function_two_keys_to_one_key(poi_num, other_children_destination_poi_index, current_children_source_poi_index, children_src_children_dest_index);
                                            reverse_path_children_src_children_dest = true;
                                        }
                                        assert(parent_src_children_dest_path[parent_src_children_dest_path.size() - 1].getx() == children_src_parent_src_path[0].getx() &&
                                               parent_src_children_dest_path[parent_src_children_dest_path.size() - 1].gety() == children_src_parent_src_path[0].gety() &&
                                               parent_src_children_dest_path[parent_src_children_dest_path.size() - 1].getz() == children_src_parent_src_path[0].getz());
                                        children_src_children_dest_distance = children_src_parent_src_distance + parent_src_children_dest_distance;
                                        for (int m = 0; m < parent_src_children_dest_path.size(); m++)
                                        {
                                            children_src_children_dest_path.push_back(parent_src_children_dest_path[m]);
                                        }
                                        for (int m = 1; m < children_src_parent_src_path.size(); m++)
                                        {
                                            children_src_children_dest_path.push_back(children_src_parent_src_path[m]);
                                        }
                                        if (reverse_path_children_src_children_dest)
                                        {
                                            std::reverse(children_src_children_dest_path.begin(), children_src_children_dest_path.end());
                                        }
                                        distance_poi_to_poi_map[children_src_children_dest_index] = children_src_children_dest_distance;
                                        path_poi_to_poi_map[children_src_children_dest_index] = children_src_children_dest_path;
                                    }
                                }
                                // store this children destination poi and need to run SSAD
                                else
                                {
                                    children_destinations_poi_list.push_back(point_cloud_geodesic::PathPoint(&point_cloud->pc_points()[poi_list[other_children_destination_poi_index]]));
                                    children_destinations_poi_index_list.push_back(other_children_destination_poi_index);
                                }
                            }
                        }

                        algorithm.propagate(children_one_source_poi_list, &children_destinations_poi_list);

                        for (int k = 0; k < children_destinations_poi_index_list.size(); k++)
                        {
                            std::vector<point_cloud_geodesic::PathPoint> path;
                            point_cloud_geodesic::PathPoint one_destination(&point_cloud->pc_points()[poi_list[children_destinations_poi_index_list[k]]]);
                            algorithm.trace_back(one_destination, path);

                            int children_src_children_dest_index2;
                            if (current_children_source_poi_index <= children_destinations_poi_index_list[k])
                            {
                                hash_function_two_keys_to_one_key(poi_num, current_children_source_poi_index, children_destinations_poi_index_list[k], children_src_children_dest_index2);
                            }
                            else
                            {
                                hash_function_two_keys_to_one_key(poi_num, children_destinations_poi_index_list[k], current_children_source_poi_index, children_src_children_dest_index2);
                                std::reverse(path.begin(), path.end());
                            }
                            distance_poi_to_poi_map[children_src_children_dest_index2] = length(path);
                            path_poi_to_poi_map[children_src_children_dest_index2] = path;
                            index_path_size += path.size();
                            index_path_num++;
                        }
                    }
                }
            }
        }
    }

    memory_usage += algorithm.get_memory() + index_path_num * sizeof(double) + index_path_size * sizeof(point_cloud_geodesic::PathPoint); //+ 0.5 * poi_num * (poi_num - 1) * sizeof(double) + pairwise_path_poi_to_poi_size * sizeof(point_cloud_geodesic::PathPoint);
    index_size += index_path_num * sizeof(double) + index_path_size * sizeof(point_cloud_geodesic::PathPoint);

    auto stop_construction_time = std::chrono::high_resolution_clock::now();
    auto duration_construction_time = std::chrono::duration_cast<std::chrono::milliseconds>(stop_construction_time - start_construction_time);
    construction_time = duration_construction_time.count();
}

void RC_Oracle_Naive(int poi_num, point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list,
                     std::unordered_map<int, double> &pairwise_distance_poi_to_poi_map,
                     std::unordered_map<int, std::vector<point_cloud_geodesic::PathPoint>> &pairwise_path_poi_to_poi_map,
                     double &construction_time, double &memory_usage, double &index_size)
{
    auto start_construction_time = std::chrono::high_resolution_clock::now();

    point_cloud_geodesic::PointCloudGeodesicAlgorithmDijkstra algorithm(point_cloud);
    double const distance_limit = point_cloud_geodesic::INFIN;
    pairwise_distance_poi_to_poi_map.clear();
    pairwise_path_poi_to_poi_map.clear();
    int pairwise_path_poi_to_poi_size = 0;
    std::vector<point_cloud_geodesic::PathPoint> one_source_poi_list;
    std::vector<point_cloud_geodesic::PathPoint> destinations_poi_list;

    for (int i = 0; i < poi_num; i++)
    {
        one_source_poi_list.clear();
        destinations_poi_list.clear();
        one_source_poi_list.push_back(point_cloud_geodesic::PathPoint(&point_cloud->pc_points()[poi_list[i]]));
        for (int j = i; j < poi_num; j++)
        {
            destinations_poi_list.push_back(point_cloud_geodesic::PathPoint(&point_cloud->pc_points()[poi_list[j]]));
        }
        algorithm.propagate(one_source_poi_list, distance_limit);
        for (int j = i; j < poi_num; j++)
        {
            std::vector<point_cloud_geodesic::PathPoint> path;
            point_cloud_geodesic::PathPoint one_destination(&point_cloud->pc_points()[poi_list[j]]);
            algorithm.trace_back(one_destination, path);
            int i_j;
            hash_function_two_keys_to_one_key(poi_num, i, j, i_j);
            pairwise_distance_poi_to_poi_map[i_j] = length(path);
            pairwise_path_poi_to_poi_map[i_j] = path;
            pairwise_path_poi_to_poi_size += path.size();
        }
    }

    memory_usage += algorithm.get_memory() + 0.5 * poi_num * (poi_num - 1) * sizeof(double) + pairwise_path_poi_to_poi_size * sizeof(point_cloud_geodesic::PathPoint);
    index_size += 0.5 * poi_num * (poi_num - 1) * sizeof(double) + pairwise_path_poi_to_poi_size * sizeof(point_cloud_geodesic::PathPoint);

    auto stop_construction_time = std::chrono::high_resolution_clock::now();
    auto duration_construction_time = std::chrono::duration_cast<std::chrono::milliseconds>(stop_construction_time - start_construction_time);
    construction_time = duration_construction_time.count();
}

void RC_Oracle_query(int poi_num, std::unordered_map<int, double> &distance_poi_to_poi_map,
                     std::unordered_map<int, std::vector<point_cloud_geodesic::PathPoint>> &path_poi_to_poi_map,
                     std::unordered_map<int, int> &non_exact_source_poi_map,
                     std::unordered_map<int, int> &exact_source_poi_process_order_map,
                     int source_poi_index, int destination_poi_index, double &distance_result,
                     std::vector<point_cloud_geodesic::PathPoint> &path_result, double &query_time)
{
    auto start_query_time = std::chrono::high_resolution_clock::now();

    int x_y;
    int current_parent_source_poi_index;
    int current_children_source_poi_index;
    int other_children_destination_poi_index;
    bool approximate_path = false;

    if (source_poi_index > destination_poi_index)
    {
        int temp = destination_poi_index;
        destination_poi_index = source_poi_index;
        source_poi_index = temp;
    }

    if ((non_exact_source_poi_map.count(source_poi_index) == 0 && non_exact_source_poi_map.count(destination_poi_index) == 0) ||
        (non_exact_source_poi_map.count(source_poi_index) == 0 && non_exact_source_poi_map.count(destination_poi_index) != 0 && exact_source_poi_process_order_map[source_poi_index] <= exact_source_poi_process_order_map[non_exact_source_poi_map[destination_poi_index]]) ||
        (non_exact_source_poi_map.count(source_poi_index) != 0 && non_exact_source_poi_map.count(destination_poi_index) == 0 && exact_source_poi_process_order_map[non_exact_source_poi_map[source_poi_index]] >= exact_source_poi_process_order_map[destination_poi_index]) ||
        (non_exact_source_poi_map.count(source_poi_index) != 0 && non_exact_source_poi_map.count(destination_poi_index) != 0 && exact_source_poi_process_order_map[non_exact_source_poi_map[source_poi_index]] == exact_source_poi_process_order_map[non_exact_source_poi_map[destination_poi_index]]))
    {
        hash_function_two_keys_to_one_key(poi_num, source_poi_index, destination_poi_index, x_y);
        if (distance_poi_to_poi_map.count(x_y) == 0)
        {
            std::cout << source_poi_index << " " << destination_poi_index << std::endl;
        }
        assert(distance_poi_to_poi_map.count(x_y) != 0);
        distance_result = distance_poi_to_poi_map[x_y];
        distance_result = round(distance_result * 1000000000.0) / 1000000000.0;
        path_result = path_poi_to_poi_map[x_y];
    }
    else if ((non_exact_source_poi_map.count(source_poi_index) == 0 && non_exact_source_poi_map.count(destination_poi_index) != 0 && exact_source_poi_process_order_map[source_poi_index] > exact_source_poi_process_order_map[non_exact_source_poi_map[destination_poi_index]]) ||
             (non_exact_source_poi_map.count(source_poi_index) != 0 && non_exact_source_poi_map.count(destination_poi_index) != 0 && exact_source_poi_process_order_map[non_exact_source_poi_map[source_poi_index]] > exact_source_poi_process_order_map[non_exact_source_poi_map[destination_poi_index]]))
    {
        current_parent_source_poi_index = non_exact_source_poi_map[destination_poi_index];
        current_children_source_poi_index = destination_poi_index;
        other_children_destination_poi_index = source_poi_index;
        approximate_path = true;
    }
    else if ((non_exact_source_poi_map.count(source_poi_index) != 0 && non_exact_source_poi_map.count(destination_poi_index) == 0 && exact_source_poi_process_order_map[non_exact_source_poi_map[source_poi_index]] < exact_source_poi_process_order_map[destination_poi_index]) ||
             (non_exact_source_poi_map.count(source_poi_index) != 0 && non_exact_source_poi_map.count(destination_poi_index) != 0 && exact_source_poi_process_order_map[non_exact_source_poi_map[source_poi_index]] < exact_source_poi_process_order_map[non_exact_source_poi_map[destination_poi_index]]))
    {
        current_parent_source_poi_index = non_exact_source_poi_map[source_poi_index];
        current_children_source_poi_index = source_poi_index;
        other_children_destination_poi_index = destination_poi_index;
        approximate_path = true;
    }
    else
    {
        assert(false);
    }

    if (approximate_path)
    {
        // the shorter part of the path between children source poi and parent source poi
        int children_src_parent_src_index;
        double children_src_parent_src_distance = 0;
        std::vector<point_cloud_geodesic::PathPoint> children_src_parent_src_path;
        children_src_parent_src_path.clear();
        bool reverse_path_children_src_parent_src = false;

        if (current_children_source_poi_index <= current_parent_source_poi_index)
        {
            hash_function_two_keys_to_one_key(poi_num, current_children_source_poi_index, current_parent_source_poi_index, children_src_parent_src_index);
        }
        else
        {
            hash_function_two_keys_to_one_key(poi_num, current_parent_source_poi_index, current_children_source_poi_index, children_src_parent_src_index);
            reverse_path_children_src_parent_src = true;
        }
        children_src_parent_src_distance = distance_poi_to_poi_map[children_src_parent_src_index];
        children_src_parent_src_path = path_poi_to_poi_map[children_src_parent_src_index];
        if (reverse_path_children_src_parent_src)
        {
            std::reverse(children_src_parent_src_path.begin(), children_src_parent_src_path.end());
        }

        // the longer part of the path between parent source poi and children destination poi
        int parent_src_children_dest_index;
        double parent_src_children_dest_distance = 0;
        std::vector<point_cloud_geodesic::PathPoint> parent_src_children_dest_path;
        parent_src_children_dest_path.clear();
        bool reverse_path_parent_src_children_dest = false;

        if (current_parent_source_poi_index <= other_children_destination_poi_index)
        {
            hash_function_two_keys_to_one_key(poi_num, current_parent_source_poi_index, other_children_destination_poi_index, parent_src_children_dest_index);
        }
        else
        {
            hash_function_two_keys_to_one_key(poi_num, other_children_destination_poi_index, current_parent_source_poi_index, parent_src_children_dest_index);
            reverse_path_parent_src_children_dest = true;
        }
        parent_src_children_dest_distance = distance_poi_to_poi_map[parent_src_children_dest_index];
        parent_src_children_dest_path = path_poi_to_poi_map[parent_src_children_dest_index];
        if (reverse_path_parent_src_children_dest)
        {
            std::reverse(parent_src_children_dest_path.begin(), parent_src_children_dest_path.end());
        }

        // sum the shorter and longer path together to be the new approximated path
        int children_src_children_dest_index;
        double children_src_children_dest_distance = 0;
        std::vector<point_cloud_geodesic::PathPoint> children_src_children_dest_path;
        children_src_children_dest_path.clear();
        bool reverse_path_children_src_children_dest = false;
        if (current_children_source_poi_index <= other_children_destination_poi_index)
        {
            hash_function_two_keys_to_one_key(poi_num, current_children_source_poi_index, other_children_destination_poi_index, children_src_children_dest_index);
        }
        else
        {
            hash_function_two_keys_to_one_key(poi_num, other_children_destination_poi_index, current_children_source_poi_index, children_src_children_dest_index);
            reverse_path_children_src_children_dest = true;
        }
        assert(parent_src_children_dest_path[parent_src_children_dest_path.size() - 1].getx() == children_src_parent_src_path[0].getx() &&
               parent_src_children_dest_path[parent_src_children_dest_path.size() - 1].gety() == children_src_parent_src_path[0].gety() &&
               parent_src_children_dest_path[parent_src_children_dest_path.size() - 1].getz() == children_src_parent_src_path[0].getz());
        children_src_children_dest_distance = children_src_parent_src_distance + parent_src_children_dest_distance;
        for (int m = 0; m < parent_src_children_dest_path.size(); m++)
        {
            children_src_children_dest_path.push_back(parent_src_children_dest_path[m]);
        }
        for (int m = 1; m < children_src_parent_src_path.size(); m++)
        {
            children_src_children_dest_path.push_back(children_src_parent_src_path[m]);
        }
        if (reverse_path_children_src_children_dest)
        {
            std::reverse(children_src_children_dest_path.begin(), children_src_children_dest_path.end());
        }
        distance_result = children_src_children_dest_distance;
        distance_result = round(distance_result * 1000000000.0) / 1000000000.0;
        path_result = children_src_children_dest_path;
    }

    auto stop_query_time = std::chrono::high_resolution_clock::now();
    auto duration_query_time = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_query_time - start_query_time);
    query_time = duration_query_time.count();
    query_time /= 1000000;
}

void RC_Oracle_Naive_query(int poi_num, std::unordered_map<int, double> &pairwise_distance_poi_to_poi_map,
                           std::unordered_map<int, std::vector<point_cloud_geodesic::PathPoint>> &pairwise_path_poi_to_poi_map,
                           int source_poi_index, int destination_poi_index, double &distance_result,
                           std::vector<point_cloud_geodesic::PathPoint> &path_result, double &query_time)
{
    auto start_query_time = std::chrono::high_resolution_clock::now();

    int x_y;
    if (source_poi_index > destination_poi_index)
    {
        int temp = destination_poi_index;
        destination_poi_index = source_poi_index;
        source_poi_index = temp;
    }
    hash_function_two_keys_to_one_key(poi_num, source_poi_index, destination_poi_index, x_y);
    distance_result = pairwise_distance_poi_to_poi_map[x_y];
    distance_result = round(distance_result * 1000000000.0) / 1000000000.0;
    path_result = pairwise_path_poi_to_poi_map[x_y];

    auto stop_query_time = std::chrono::high_resolution_clock::now();
    auto duration_query_time = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_query_time - start_query_time);
    query_time = duration_query_time.count();
    query_time /= 1000000;
}

void RC_Oracle_all_poi_knn_or_range_query(int poi_num, std::unordered_map<int, double> &distance_poi_to_poi_map,
                                          std::unordered_map<int, int> &non_exact_source_poi_map,
                                          std::unordered_map<int, int> &exact_source_poi_process_order_map,
                                          int knn_one_range_two, int k_value, double range,
                                          double &knn_or_range_query_time, std::vector<std::vector<int>> &all_poi_knn_or_range_list)
{
    auto start_knn_or_range_query_time = std::chrono::high_resolution_clock::now();

    std::vector<std::vector<std::pair<double, int>>> poi_to_other_poi_distance_and_index_list;
    poi_to_other_poi_distance_and_index_list.clear();
    std::vector<std::pair<double, int>> one_poi_to_other_poi_distance_and_index_list;
    for (int i = 0; i < poi_num; i++)
    {
        one_poi_to_other_poi_distance_and_index_list.clear();
        for (int j = 0; j < poi_num; j++)
        {
            int i_j;
            int source_poi_index;
            int destination_poi_index;
            int current_parent_source_poi_index;
            int current_children_source_poi_index;
            int other_children_destination_poi_index;
            bool approximate_path = false;

            if (i <= j)
            {
                source_poi_index = i;
                destination_poi_index = j;
            }
            else
            {
                source_poi_index = j;
                destination_poi_index = i;
            }
            double distance_result;
            if (source_poi_index == destination_poi_index)
            {
                distance_result = 0;
            }
            else if ((non_exact_source_poi_map.count(source_poi_index) == 0 && non_exact_source_poi_map.count(destination_poi_index) == 0) ||
                     (non_exact_source_poi_map.count(source_poi_index) == 0 && non_exact_source_poi_map.count(destination_poi_index) != 0 && exact_source_poi_process_order_map[source_poi_index] <= exact_source_poi_process_order_map[non_exact_source_poi_map[destination_poi_index]]) ||
                     (non_exact_source_poi_map.count(source_poi_index) != 0 && non_exact_source_poi_map.count(destination_poi_index) == 0 && exact_source_poi_process_order_map[non_exact_source_poi_map[source_poi_index]] >= exact_source_poi_process_order_map[destination_poi_index]) ||
                     (non_exact_source_poi_map.count(source_poi_index) != 0 && non_exact_source_poi_map.count(destination_poi_index) != 0 && exact_source_poi_process_order_map[non_exact_source_poi_map[source_poi_index]] == exact_source_poi_process_order_map[non_exact_source_poi_map[destination_poi_index]]))
            {
                hash_function_two_keys_to_one_key(poi_num, source_poi_index, destination_poi_index, i_j);
                if (distance_poi_to_poi_map.count(i_j) == 0)
                {
                    std::cout << "i: " << i << ", j: " << j << std::endl;
                }
                assert(distance_poi_to_poi_map.count(i_j) != 0);
                distance_result = distance_poi_to_poi_map[i_j];
            }
            else if ((non_exact_source_poi_map.count(source_poi_index) == 0 && non_exact_source_poi_map.count(destination_poi_index) != 0 && exact_source_poi_process_order_map[source_poi_index] > exact_source_poi_process_order_map[non_exact_source_poi_map[destination_poi_index]]) ||
                     (non_exact_source_poi_map.count(source_poi_index) != 0 && non_exact_source_poi_map.count(destination_poi_index) != 0 && exact_source_poi_process_order_map[non_exact_source_poi_map[source_poi_index]] > exact_source_poi_process_order_map[non_exact_source_poi_map[destination_poi_index]]))
            {
                current_parent_source_poi_index = non_exact_source_poi_map[destination_poi_index];
                current_children_source_poi_index = destination_poi_index;
                other_children_destination_poi_index = source_poi_index;
                approximate_path = true;
            }
            else if ((non_exact_source_poi_map.count(source_poi_index) != 0 && non_exact_source_poi_map.count(destination_poi_index) == 0 && exact_source_poi_process_order_map[non_exact_source_poi_map[source_poi_index]] < exact_source_poi_process_order_map[destination_poi_index]) ||
                     (non_exact_source_poi_map.count(source_poi_index) != 0 && non_exact_source_poi_map.count(destination_poi_index) != 0 && exact_source_poi_process_order_map[non_exact_source_poi_map[source_poi_index]] < exact_source_poi_process_order_map[non_exact_source_poi_map[destination_poi_index]]))
            {
                current_parent_source_poi_index = non_exact_source_poi_map[source_poi_index];
                current_children_source_poi_index = source_poi_index;
                other_children_destination_poi_index = destination_poi_index;
                approximate_path = true;
            }
            else
            {
                std::cout << "source_poi_index: " << source_poi_index << ", destination_poi_index: " << destination_poi_index << std::endl;
                assert(false);
            }

            if (approximate_path)
            {
                // the shorter part of the path between children source poi and parent source poi
                int children_src_parent_src_index;
                double children_src_parent_src_distance = 0;

                if (current_children_source_poi_index <= current_parent_source_poi_index)
                {
                    hash_function_two_keys_to_one_key(poi_num, current_children_source_poi_index, current_parent_source_poi_index, children_src_parent_src_index);
                }
                else
                {
                    hash_function_two_keys_to_one_key(poi_num, current_parent_source_poi_index, current_children_source_poi_index, children_src_parent_src_index);
                }
                children_src_parent_src_distance = distance_poi_to_poi_map[children_src_parent_src_index];

                // the longer part of the path between parent source poi and children destination poi
                int parent_src_children_dest_index;
                double parent_src_children_dest_distance = 0;

                if (current_parent_source_poi_index <= other_children_destination_poi_index)
                {
                    hash_function_two_keys_to_one_key(poi_num, current_parent_source_poi_index, other_children_destination_poi_index, parent_src_children_dest_index);
                }
                else
                {
                    hash_function_two_keys_to_one_key(poi_num, other_children_destination_poi_index, current_parent_source_poi_index, parent_src_children_dest_index);
                }
                parent_src_children_dest_distance = distance_poi_to_poi_map[parent_src_children_dest_index];

                // sum the shorter and longer path together to be the new approximated path
                int children_src_children_dest_index;
                double children_src_children_dest_distance = 0;
                if (current_children_source_poi_index <= other_children_destination_poi_index)
                {
                    hash_function_two_keys_to_one_key(poi_num, current_children_source_poi_index, other_children_destination_poi_index, children_src_children_dest_index);
                }
                else
                {
                    hash_function_two_keys_to_one_key(poi_num, other_children_destination_poi_index, current_children_source_poi_index, children_src_children_dest_index);
                }
                children_src_children_dest_distance = children_src_parent_src_distance + parent_src_children_dest_distance;
                distance_result = children_src_children_dest_distance;
            }

            one_poi_to_other_poi_distance_and_index_list.push_back(std::make_pair(distance_result, j));
        }
        std::sort(one_poi_to_other_poi_distance_and_index_list.begin(), one_poi_to_other_poi_distance_and_index_list.end());
        poi_to_other_poi_distance_and_index_list.push_back(one_poi_to_other_poi_distance_and_index_list);
    }
    knn_or_range_query(knn_one_range_two, k_value, range, poi_to_other_poi_distance_and_index_list, all_poi_knn_or_range_list);

    auto stop_knn_or_range_query_time = std::chrono::high_resolution_clock::now();
    auto duration_knn_or_range_query_time = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_knn_or_range_query_time - start_knn_or_range_query_time);
    knn_or_range_query_time = duration_knn_or_range_query_time.count();
    knn_or_range_query_time /= 1000000;
}

void RC_Oracle_Naive_all_poi_knn_or_range_query(int poi_num, std::unordered_map<int, double> &pairwise_distance_poi_to_poi_map,
                                                int knn_one_range_two, int k_value, double range, double &knn_or_range_query_time,
                                                std::vector<std::vector<int>> &all_poi_knn_or_range_list)
{
    auto start_knn_or_range_query_time = std::chrono::high_resolution_clock::now();

    std::vector<std::vector<std::pair<double, int>>> poi_to_other_poi_distance_and_index_list;
    poi_to_other_poi_distance_and_index_list.clear();
    std::vector<std::pair<double, int>> one_poi_to_other_poi_distance_and_index_list;
    for (int i = 0; i < poi_num; i++)
    {
        one_poi_to_other_poi_distance_and_index_list.clear();
        for (int j = 0; j < poi_num; j++)
        {
            int i_j;
            if (i <= j)
            {
                hash_function_two_keys_to_one_key(poi_num, i, j, i_j);
            }
            else
            {
                hash_function_two_keys_to_one_key(poi_num, j, i, i_j);
            }
            one_poi_to_other_poi_distance_and_index_list.push_back(std::make_pair(pairwise_distance_poi_to_poi_map[i_j], j));
        }
        std::sort(one_poi_to_other_poi_distance_and_index_list.begin(), one_poi_to_other_poi_distance_and_index_list.end());
        poi_to_other_poi_distance_and_index_list.push_back(one_poi_to_other_poi_distance_and_index_list);
    }
    knn_or_range_query(knn_one_range_two, k_value, range, poi_to_other_poi_distance_and_index_list, all_poi_knn_or_range_list);

    auto stop_knn_or_range_query_time = std::chrono::high_resolution_clock::now();
    auto duration_knn_or_range_query_time = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_knn_or_range_query_time - start_knn_or_range_query_time);
    knn_or_range_query_time = duration_knn_or_range_query_time.count();
    knn_or_range_query_time /= 1000000;
}

void SE_Oracle_FastFly_Adapt(
    int poi_num, point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list, double epsilon,
    int source_poi_index, int destination_poi_index, double &construction_time,
    double &query_time, double &memory_usage, double &index_size, double &distance_result,
    std::vector<point_cloud_geodesic::PathPoint> &path_result, bool run_knn_query, bool run_range_query,
    int k_value, double range, double &knn_query_time, std::vector<std::vector<int>> &all_poi_knn_query_list,
    double &range_query_time, std::vector<std::vector<int>> &all_poi_range_list)
{
    auto start_construction_time = std::chrono::high_resolution_clock::now();

    std::unordered_map<int, double> pre_pairwise_distance_poi_to_poi_map;
    std::unordered_map<int, std::vector<point_cloud_geodesic::PathPoint>> pre_pairwise_path_poi_to_poi_map;
    double pre_pairwise_memory_usage = 0;

    pre_compute_Point(poi_num, point_cloud, poi_list, pre_pairwise_distance_poi_to_poi_map, pre_pairwise_path_poi_to_poi_map, pre_pairwise_memory_usage);

    int geo_tree_node_id = 1; // the root node has 0 id
    std::unordered_map<int, GeoPair_C *> geopairs;
    geopairs.clear();
    std::vector<GeoNode *> all_poi;
    all_poi.clear();
    std::vector<std::pair<int, GeoNode *>> pois;
    pois.clear();
    std::unordered_map<int, int> poi_unordered_map;
    poi_unordered_map.clear();

    for (int i = 0; i < poi_num; i++)
    {
        GeoNode *n = new GeoNode(poi_list[i], 0);
        all_poi.push_back(n);
        std::pair<int, GeoNode *> m(poi_list[i], n);
        pois.push_back(m);
        poi_unordered_map[poi_list[i]] = i;
    }

    double radius = 0;
    stx::btree<int, GeoNode *> pois_B_tree(pois.begin(), pois.end());

    for (int i = 0; i < poi_num; i++)
    {
        int x_in_poi_list = 0;
        int y_in_poi_list = i;
        int x_y_in_poi_list;
        if (x_in_poi_list <= y_in_poi_list)
        {
            hash_function_two_keys_to_one_key(poi_num, x_in_poi_list, y_in_poi_list, x_y_in_poi_list);
        }
        else
        {
            hash_function_two_keys_to_one_key(poi_num, y_in_poi_list, x_in_poi_list, x_y_in_poi_list);
        }
        radius = std::max(pre_pairwise_distance_poi_to_poi_map[x_y_in_poi_list], radius);
    }
    GeoNode *root_geo = new GeoNode(0, poi_list[0], radius);

    stx::btree<int, GeoNode *> pois_as_center_each_parent_layer;
    pois_as_center_each_parent_layer.clear();
    build_geo_tree_C(geo_tree_node_id, point_cloud, *root_geo, poi_num, pois_B_tree, pois_as_center_each_parent_layer, pre_pairwise_distance_poi_to_poi_map, pre_pairwise_path_poi_to_poi_map, poi_unordered_map);

    std::vector<GeoNode *> partition_tree_to_compressed_partition_tree_to_be_removed_nodes;
    partition_tree_to_compressed_partition_tree_to_be_removed_nodes.clear();
    std::unordered_map<int, GeoNode *> geo_node_in_partition_tree_unordered_map;
    geo_node_in_partition_tree_unordered_map.clear();
    partition_tree_to_compressed_partition_tree(*root_geo, partition_tree_to_compressed_partition_tree_to_be_removed_nodes, geo_node_in_partition_tree_unordered_map);

    std::unordered_map<int, int> geo_pair_unordered_map;
    geo_pair_unordered_map.clear();
    int pairwise_path_poi_to_poi_size = 0;
    int WSPD_oracle_edge_num = 0;
    double WSPD_oracle_epsilon = pow((1 + epsilon), 0.35) - 1;
    generate_geo_pair_C(geo_tree_node_id, WSPD_oracle_edge_num, point_cloud, *root_geo, *root_geo, WSPD_oracle_epsilon, geopairs, poi_unordered_map, geo_pair_unordered_map, pre_pairwise_distance_poi_to_poi_map, pre_pairwise_path_poi_to_poi_map, pairwise_path_poi_to_poi_size);
    index_size = WSPD_oracle_edge_num * sizeof(double) + pairwise_path_poi_to_poi_size * sizeof(geodesic::SurfacePoint);

    memory_usage += pre_pairwise_memory_usage + (geo_tree_node_id + 1) * sizeof(GeoNode) + WSPD_oracle_edge_num * sizeof(double) + pairwise_path_poi_to_poi_size * sizeof(geodesic::SurfacePoint);

    auto stop_construction_time = std::chrono::high_resolution_clock::now();
    auto duration_construction_time = std::chrono::duration_cast<std::chrono::milliseconds>(stop_construction_time - start_construction_time);
    construction_time = duration_construction_time.count();

    auto start_query_time = std::chrono::high_resolution_clock::now();

    int a, b;
    distance_result = query_geo_C(geo_tree_node_id, *geo_node_in_partition_tree_unordered_map[all_poi[source_poi_index]->index], *geo_node_in_partition_tree_unordered_map[all_poi[destination_poi_index]->index], a, b, geopairs, path_result);
    distance_result = round(distance_result * 1000000000.0) / 1000000000.0;

    auto stop_query_time = std::chrono::high_resolution_clock::now();
    auto duration_query_time = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_query_time - start_query_time);
    query_time = duration_query_time.count();
    query_time /= 1000000;

    auto start_knn_query_time = std::chrono::high_resolution_clock::now();

    if (run_knn_query)
    {
        all_poi_knn_or_range_query_geo_C(poi_num, geo_tree_node_id, geo_node_in_partition_tree_unordered_map,
                                         all_poi, geopairs, 1, k_value, range, all_poi_knn_query_list);
    }

    auto stop_knn_query_time = std::chrono::high_resolution_clock::now();
    auto duration_knn_query_time = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_knn_query_time - start_knn_query_time);
    knn_query_time = duration_knn_query_time.count();
    knn_query_time /= 1000000;

    auto start_range_query_time = std::chrono::high_resolution_clock::now();

    if (run_range_query)
    {
        all_poi_knn_or_range_query_geo_C(poi_num, geo_tree_node_id, geo_node_in_partition_tree_unordered_map,
                                         all_poi, geopairs, 2, k_value, range, all_poi_range_list);
    }

    auto stop_range_query_time = std::chrono::high_resolution_clock::now();
    auto duration_range_query_time = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_range_query_time - start_range_query_time);
    range_query_time = duration_range_query_time.count();
    range_query_time /= 1000000;
}

void SE_Oracle_Adapt(
    int poi_num, point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list, double epsilon,
    int source_poi_index, int destination_poi_index,
    double &point_cloud_to_terrain_time, double &construction_time,
    double &query_time, double &point_cloud_to_terrain_memory_usage,
    double &memory_usage, double &index_size, double &distance_result,
    std::vector<geodesic::SurfacePoint> &path_result, bool run_knn_query, bool run_range_query,
    int k_value, double range, double &knn_query_time, std::vector<std::vector<int>> &all_poi_knn_query_list,
    double &range_query_time, std::vector<std::vector<int>> &all_poi_range_list)
{
    geodesic::Mesh mesh;
    point_cloud_to_terrain_and_initialize_terrain(point_cloud, &mesh, point_cloud_to_terrain_time, point_cloud_to_terrain_memory_usage);

    auto start_construction_time = std::chrono::high_resolution_clock::now();

    std::unordered_map<int, double> pre_pairwise_distance_poi_to_poi_map;
    std::unordered_map<int, std::vector<geodesic::SurfacePoint>> pre_pairwise_path_poi_to_poi_map;
    double pre_pairwise_memory_usage = 0;

    pre_compute_FaceExact(poi_num, &mesh, poi_list, pre_pairwise_distance_poi_to_poi_map, pre_pairwise_path_poi_to_poi_map, pre_pairwise_memory_usage);

    int geo_tree_node_id = 1; // the root node has 0 id
    std::unordered_map<int, GeoPair_T *> geopairs;
    geopairs.clear();
    std::vector<GeoNode *> all_poi;
    all_poi.clear();
    std::vector<std::pair<int, GeoNode *>> pois;
    pois.clear();
    std::unordered_map<int, int> poi_unordered_map;
    poi_unordered_map.clear();

    for (int i = 0; i < poi_num; i++)
    {
        GeoNode *n = new GeoNode(poi_list[i], 0);
        all_poi.push_back(n);
        std::pair<int, GeoNode *> m(poi_list[i], n);
        pois.push_back(m);
        poi_unordered_map[poi_list[i]] = i;
    }

    double radius = 0;
    stx::btree<int, GeoNode *> pois_B_tree(pois.begin(), pois.end());

    for (int i = 0; i < poi_num; i++)
    {
        int x_in_poi_list = 0;
        int y_in_poi_list = i;
        int x_y_in_poi_list;
        if (x_in_poi_list <= y_in_poi_list)
        {
            hash_function_two_keys_to_one_key(poi_num, x_in_poi_list, y_in_poi_list, x_y_in_poi_list);
        }
        else
        {
            hash_function_two_keys_to_one_key(poi_num, y_in_poi_list, x_in_poi_list, x_y_in_poi_list);
        }
        radius = std::max(pre_pairwise_distance_poi_to_poi_map[x_y_in_poi_list], radius);
    }
    GeoNode *root_geo = new GeoNode(0, poi_list[0], radius);

    stx::btree<int, GeoNode *> pois_as_center_each_parent_layer;
    pois_as_center_each_parent_layer.clear();
    build_geo_tree_T(geo_tree_node_id, &mesh, *root_geo, poi_num, pois_B_tree, pois_as_center_each_parent_layer, pre_pairwise_distance_poi_to_poi_map, pre_pairwise_path_poi_to_poi_map, poi_unordered_map);

    std::vector<GeoNode *> partition_tree_to_compressed_partition_tree_to_be_removed_nodes;
    partition_tree_to_compressed_partition_tree_to_be_removed_nodes.clear();
    std::unordered_map<int, GeoNode *> geo_node_in_partition_tree_unordered_map;
    geo_node_in_partition_tree_unordered_map.clear();
    partition_tree_to_compressed_partition_tree(*root_geo, partition_tree_to_compressed_partition_tree_to_be_removed_nodes, geo_node_in_partition_tree_unordered_map);

    std::unordered_map<int, int> geo_pair_unordered_map;
    geo_pair_unordered_map.clear();
    int pairwise_path_poi_to_poi_size = 0;
    int WSPD_oracle_edge_num = 0;
    double WSPD_oracle_epsilon = pow((1 + epsilon), 0.35) - 1;
    generate_geo_pair_T(geo_tree_node_id, WSPD_oracle_edge_num, &mesh, *root_geo, *root_geo, WSPD_oracle_epsilon, geopairs, poi_unordered_map, geo_pair_unordered_map, pre_pairwise_distance_poi_to_poi_map, pre_pairwise_path_poi_to_poi_map, pairwise_path_poi_to_poi_size);
    index_size = WSPD_oracle_edge_num * sizeof(double) + pairwise_path_poi_to_poi_size * sizeof(geodesic::SurfacePoint);

    memory_usage += pre_pairwise_memory_usage + (geo_tree_node_id + 1) * sizeof(GeoNode) + WSPD_oracle_edge_num * sizeof(double) + pairwise_path_poi_to_poi_size * sizeof(geodesic::SurfacePoint);

    auto stop_construction_time = std::chrono::high_resolution_clock::now();
    auto duration_construction_time = std::chrono::duration_cast<std::chrono::milliseconds>(stop_construction_time - start_construction_time);
    construction_time = duration_construction_time.count();

    auto start_query_time = std::chrono::high_resolution_clock::now();

    int a, b;
    distance_result = query_geo_T(geo_tree_node_id, *geo_node_in_partition_tree_unordered_map[all_poi[source_poi_index]->index], *geo_node_in_partition_tree_unordered_map[all_poi[destination_poi_index]->index], a, b, geopairs, path_result);
    distance_result = round(distance_result * 1000000000.0) / 1000000000.0;

    auto stop_query_time = std::chrono::high_resolution_clock::now();
    auto duration_query_time = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_query_time - start_query_time);
    query_time = duration_query_time.count();
    query_time /= 1000000;

    auto start_knn_query_time = std::chrono::high_resolution_clock::now();

    if (run_knn_query)
    {
        all_poi_knn_or_range_query_geo_T(poi_num, geo_tree_node_id, geo_node_in_partition_tree_unordered_map,
                                         all_poi, geopairs, 1, k_value, range, all_poi_knn_query_list);
    }

    auto stop_knn_query_time = std::chrono::high_resolution_clock::now();
    auto duration_knn_query_time = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_knn_query_time - start_knn_query_time);
    knn_query_time = duration_knn_query_time.count();
    knn_query_time /= 1000000;

    auto start_range_query_time = std::chrono::high_resolution_clock::now();

    if (run_range_query)
    {
        all_poi_knn_or_range_query_geo_T(poi_num, geo_tree_node_id, geo_node_in_partition_tree_unordered_map,
                                         all_poi, geopairs, 2, k_value, range, all_poi_range_list);
    }

    auto stop_range_query_time = std::chrono::high_resolution_clock::now();
    auto duration_range_query_time = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_range_query_time - start_range_query_time);
    range_query_time = duration_range_query_time.count();
    range_query_time /= 1000000;
}

void EAR_Oracle_FastFly_Adapt(
    int sqrt_num_of_box, int poi_num, point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list, double epsilon,
    int source_poi_index, int destination_poi_index,
    double &construction_time, double &query_time,
    double &memory_usage, double &index_size, double &distance_result,
    std::vector<point_cloud_geodesic::PathPoint> &path_result, bool run_knn_query, bool run_range_query,
    int k_value, double range, double &knn_query_time, std::vector<std::vector<int>> &all_poi_knn_query_list,
    double &range_query_time, std::vector<std::vector<int>> &all_poi_range_list)
{
    auto start_construction_time = std::chrono::high_resolution_clock::now();

    std::unordered_map<int, int> highway_node_id_map;
    std::unordered_map<int, std::unordered_map<int, int>> highway_node_id_with_box_id_map;
    highway_node_id_map.clear();
    highway_node_id_with_box_id_map.clear();
    geodesic::Mesh mesh;
    double point_cloud_to_terrain_time, point_cloud_to_terrain_memory_usage;
    point_cloud_to_terrain_and_initialize_terrain(point_cloud, &mesh, point_cloud_to_terrain_time, point_cloud_to_terrain_memory_usage);
    divide_mesh_into_box(&mesh, sqrt_num_of_box, highway_node_id_map, highway_node_id_with_box_id_map);

    std::vector<int> highway_node_list;
    highway_node_list.clear();
    for (auto i : highway_node_id_map)
    {
        highway_node_list.push_back(i.first);
    }

    std::unordered_map<int, double> pre_distance_highway_node_to_highway_node_map;
    std::unordered_map<int, std::vector<point_cloud_geodesic::PathPoint>> pre_path_highway_node_to_highway_node_map;
    double pre_memory_usage = 0;

    pre_compute_EAR_Oracle_highway_node_C(point_cloud, highway_node_list, pre_distance_highway_node_to_highway_node_map,
                                          pre_path_highway_node_to_highway_node_map, pre_memory_usage);

    int geo_tree_node_id = 1; // the root node has 0 id
    std::unordered_map<int, GeoPair_C *> geopairs;
    geopairs.clear();
    std::vector<GeoNode *> all_highway_node;
    all_highway_node.clear();
    std::vector<std::pair<int, GeoNode *>> highway_nodes;
    highway_nodes.clear();
    std::unordered_map<int, int> highway_node_unordered_map;
    highway_node_unordered_map.clear();

    for (int i = 0; i < highway_node_list.size(); i++)
    {
        GeoNode *n = new GeoNode(highway_node_list[i], 0);
        all_highway_node.push_back(n);
        std::pair<int, GeoNode *> m(highway_node_list[i], n);
        highway_nodes.push_back(m);
        highway_node_unordered_map[highway_node_list[i]] = i;
    }

    double radius = 0;
    stx::btree<int, GeoNode *> highway_nodes_B_tree(highway_nodes.begin(), highway_nodes.end());

    for (int i = 0; i < highway_node_list.size(); i++)
    {
        int x_in_highway_node_list = 0;
        int y_in_highway_node_list = i;
        int x_y_in_highway_node_list;
        if (x_in_highway_node_list <= y_in_highway_node_list)
        {
            hash_function_two_keys_to_one_key(highway_node_list.size(), x_in_highway_node_list, y_in_highway_node_list, x_y_in_highway_node_list);
        }
        else
        {
            hash_function_two_keys_to_one_key(highway_node_list.size(), y_in_highway_node_list, x_in_highway_node_list, x_y_in_highway_node_list);
        }
        radius = std::max(pre_distance_highway_node_to_highway_node_map[x_y_in_highway_node_list], radius);
    }
    GeoNode *root_geo = new GeoNode(0, highway_node_list[0], radius);

    stx::btree<int, GeoNode *> highway_nodes_as_center_each_parent_layer;
    highway_nodes_as_center_each_parent_layer.clear();
    build_geo_tree_C(geo_tree_node_id, point_cloud, *root_geo, highway_node_list.size(), highway_nodes_B_tree, highway_nodes_as_center_each_parent_layer, pre_distance_highway_node_to_highway_node_map, pre_path_highway_node_to_highway_node_map, highway_node_unordered_map);

    std::vector<GeoNode *> partition_tree_to_compressed_partition_tree_to_be_removed_nodes;
    partition_tree_to_compressed_partition_tree_to_be_removed_nodes.clear();
    std::unordered_map<int, GeoNode *> geo_node_in_partition_tree_unordered_map;
    geo_node_in_partition_tree_unordered_map.clear();
    partition_tree_to_compressed_partition_tree(*root_geo, partition_tree_to_compressed_partition_tree_to_be_removed_nodes, geo_node_in_partition_tree_unordered_map);

    std::unordered_map<int, int> geo_pair_unordered_map;
    geo_pair_unordered_map.clear();
    int EAR_oracle_edge_num = 0;
    double EAR_oracle_epsilon = pow((1 + epsilon), 0.35) - 1;
    int pairwise_path_highway_node_to_highway_node_size = 0;
    generate_geo_pair_C(geo_tree_node_id, EAR_oracle_edge_num, point_cloud, *root_geo, *root_geo, epsilon, geopairs, highway_node_unordered_map, geo_pair_unordered_map, pre_distance_highway_node_to_highway_node_map, pre_path_highway_node_to_highway_node_map, pairwise_path_highway_node_to_highway_node_size);
    index_size = EAR_oracle_edge_num * sizeof(double) + pairwise_path_highway_node_to_highway_node_size * sizeof(point_cloud_geodesic::PathPoint);

    std::unordered_map<int, double> distance_poi_to_highway_node_map;
    std::unordered_map<int, std::vector<point_cloud_geodesic::PathPoint>> path_poi_to_highway_node_map;

    poi_to_highway_node_path_C(point_cloud, &mesh, sqrt_num_of_box, highway_node_id_with_box_id_map, poi_list, distance_poi_to_highway_node_map, path_poi_to_highway_node_map, index_size, memory_usage);

    memory_usage += pre_memory_usage + (geo_tree_node_id + 1) * sizeof(GeoNode) + EAR_oracle_edge_num * sizeof(double) + pairwise_path_highway_node_to_highway_node_size * sizeof(point_cloud_geodesic::PathPoint);

    auto stop_construction_time = std::chrono::high_resolution_clock::now();
    auto duration_construction_time = std::chrono::duration_cast<std::chrono::milliseconds>(stop_construction_time - start_construction_time);
    construction_time = duration_construction_time.count();

    auto start_query_time = std::chrono::high_resolution_clock::now();

    EAR_Oracle_query_C(point_cloud, &mesh, poi_list, sqrt_num_of_box, geo_tree_node_id, highway_node_id_with_box_id_map,
                       all_highway_node, geo_node_in_partition_tree_unordered_map,
                       geopairs, distance_poi_to_highway_node_map, path_poi_to_highway_node_map,
                       source_poi_index, destination_poi_index, distance_result, path_result);
    distance_result = round(distance_result * 1000000000.0) / 1000000000.0;

    auto stop_query_time = std::chrono::high_resolution_clock::now();
    auto duration_query_time = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_query_time - start_query_time);
    query_time = duration_query_time.count();
    query_time /= 1000000;

    auto start_knn_query_time = std::chrono::high_resolution_clock::now();

    if (run_knn_query)
    {
        all_poi_knn_or_range_query_EAR_oracle_C(point_cloud, &mesh, poi_list, sqrt_num_of_box, geo_tree_node_id, highway_node_id_with_box_id_map,
                                                all_highway_node, geo_node_in_partition_tree_unordered_map,
                                                geopairs, distance_poi_to_highway_node_map, path_poi_to_highway_node_map,
                                                poi_num, 1, k_value, range, all_poi_knn_query_list);
    }

    auto stop_knn_query_time = std::chrono::high_resolution_clock::now();
    auto duration_knn_query_time = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_knn_query_time - start_knn_query_time);
    knn_query_time = duration_knn_query_time.count();
    knn_query_time /= 1000000;

    auto start_range_query_time = std::chrono::high_resolution_clock::now();

    if (run_range_query)
    {
        all_poi_knn_or_range_query_EAR_oracle_C(point_cloud, &mesh, poi_list, sqrt_num_of_box, geo_tree_node_id, highway_node_id_with_box_id_map,
                                                all_highway_node, geo_node_in_partition_tree_unordered_map,
                                                geopairs, distance_poi_to_highway_node_map, path_poi_to_highway_node_map,
                                                poi_num, 2, k_value, range, all_poi_knn_query_list);
    }

    auto stop_range_query_time = std::chrono::high_resolution_clock::now();
    auto duration_range_query_time = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_range_query_time - start_range_query_time);
    range_query_time = duration_range_query_time.count();
    range_query_time /= 1000000;
}

void EAR_Oracle_Adapt(
    int sqrt_num_of_box, int poi_num, point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list, double epsilon,
    int source_poi_index, int destination_poi_index,
    double &point_cloud_to_terrain_time, double &construction_time,
    double &query_time, double &point_cloud_to_terrain_memory_usage,
    double &memory_usage, double &index_size, double &distance_result,
    std::vector<geodesic::SurfacePoint> &path_result, bool run_knn_query, bool run_range_query,
    int k_value, double range, double &knn_query_time, std::vector<std::vector<int>> &all_poi_knn_query_list,
    double &range_query_time, std::vector<std::vector<int>> &all_poi_range_list)
{
    geodesic::Mesh mesh;
    point_cloud_to_terrain_and_initialize_terrain(point_cloud, &mesh, point_cloud_to_terrain_time, point_cloud_to_terrain_memory_usage);

    auto start_construction_time = std::chrono::high_resolution_clock::now();

    std::unordered_map<int, int> highway_node_id_map;
    std::unordered_map<int, std::unordered_map<int, int>> highway_node_id_with_box_id_map;
    highway_node_id_map.clear();
    highway_node_id_with_box_id_map.clear();
    divide_mesh_into_box(&mesh, sqrt_num_of_box, highway_node_id_map, highway_node_id_with_box_id_map);

    std::vector<int> highway_node_list;
    highway_node_list.clear();
    for (auto i : highway_node_id_map)
    {
        highway_node_list.push_back(i.first);
    }

    std::unordered_map<int, double> pre_distance_highway_node_to_highway_node_map;
    std::unordered_map<int, std::vector<geodesic::SurfacePoint>> pre_path_highway_node_to_highway_node_map;
    double pre_memory_usage = 0;

    pre_compute_EAR_Oracle_highway_node_T(&mesh, highway_node_list, pre_distance_highway_node_to_highway_node_map,
                                          pre_path_highway_node_to_highway_node_map, pre_memory_usage);

    int geo_tree_node_id = 1; // the root node has 0 id
    std::unordered_map<int, GeoPair_T *> geopairs;
    geopairs.clear();
    std::vector<GeoNode *> all_highway_node;
    all_highway_node.clear();
    std::vector<std::pair<int, GeoNode *>> highway_nodes;
    highway_nodes.clear();
    std::unordered_map<int, int> highway_node_unordered_map;
    highway_node_unordered_map.clear();

    for (int i = 0; i < highway_node_list.size(); i++)
    {
        GeoNode *n = new GeoNode(highway_node_list[i], 0);
        all_highway_node.push_back(n);
        std::pair<int, GeoNode *> m(highway_node_list[i], n);
        highway_nodes.push_back(m);
        highway_node_unordered_map[highway_node_list[i]] = i;
    }

    double radius = 0;
    stx::btree<int, GeoNode *> highway_nodes_B_tree(highway_nodes.begin(), highway_nodes.end());

    for (int i = 0; i < highway_node_list.size(); i++)
    {
        int x_in_highway_node_list = 0;
        int y_in_highway_node_list = i;
        int x_y_in_highway_node_list;
        if (x_in_highway_node_list <= y_in_highway_node_list)
        {
            hash_function_two_keys_to_one_key(highway_node_list.size(), x_in_highway_node_list, y_in_highway_node_list, x_y_in_highway_node_list);
        }
        else
        {
            hash_function_two_keys_to_one_key(highway_node_list.size(), y_in_highway_node_list, x_in_highway_node_list, x_y_in_highway_node_list);
        }
        radius = std::max(pre_distance_highway_node_to_highway_node_map[x_y_in_highway_node_list], radius);
    }
    GeoNode *root_geo = new GeoNode(0, highway_node_list[0], radius);

    stx::btree<int, GeoNode *> highway_nodes_as_center_each_parent_layer;
    highway_nodes_as_center_each_parent_layer.clear();
    build_geo_tree_T(geo_tree_node_id, &mesh, *root_geo, highway_node_list.size(), highway_nodes_B_tree, highway_nodes_as_center_each_parent_layer, pre_distance_highway_node_to_highway_node_map, pre_path_highway_node_to_highway_node_map, highway_node_unordered_map);

    std::vector<GeoNode *> partition_tree_to_compressed_partition_tree_to_be_removed_nodes;
    partition_tree_to_compressed_partition_tree_to_be_removed_nodes.clear();
    std::unordered_map<int, GeoNode *> geo_node_in_partition_tree_unordered_map;
    geo_node_in_partition_tree_unordered_map.clear();
    partition_tree_to_compressed_partition_tree(*root_geo, partition_tree_to_compressed_partition_tree_to_be_removed_nodes, geo_node_in_partition_tree_unordered_map);

    std::unordered_map<int, int> geo_pair_unordered_map;
    geo_pair_unordered_map.clear();
    int EAR_oracle_edge_num = 0;
    double EAR_oracle_epsilon = pow((1 + epsilon), 0.35) - 1;
    int pairwise_path_highway_node_to_highway_node_size = 0;
    generate_geo_pair_T(geo_tree_node_id, EAR_oracle_edge_num, &mesh, *root_geo, *root_geo, epsilon, geopairs, highway_node_unordered_map, geo_pair_unordered_map, pre_distance_highway_node_to_highway_node_map, pre_path_highway_node_to_highway_node_map, pairwise_path_highway_node_to_highway_node_size);
    index_size = EAR_oracle_edge_num * sizeof(double) + pairwise_path_highway_node_to_highway_node_size * sizeof(geodesic::SurfacePoint);

    std::unordered_map<int, double> distance_poi_to_highway_node_map;
    std::unordered_map<int, std::vector<geodesic::SurfacePoint>> path_poi_to_highway_node_map;

    poi_to_highway_node_path_T(&mesh, sqrt_num_of_box, highway_node_id_with_box_id_map, poi_list, distance_poi_to_highway_node_map, path_poi_to_highway_node_map, index_size, memory_usage);

    memory_usage += pre_memory_usage + (geo_tree_node_id + 1) * sizeof(GeoNode) + EAR_oracle_edge_num * sizeof(double) + pairwise_path_highway_node_to_highway_node_size * sizeof(geodesic::SurfacePoint);

    auto stop_construction_time = std::chrono::high_resolution_clock::now();
    auto duration_construction_time = std::chrono::duration_cast<std::chrono::milliseconds>(stop_construction_time - start_construction_time);
    construction_time = duration_construction_time.count();

    auto start_query_time = std::chrono::high_resolution_clock::now();

    EAR_Oracle_query_T(&mesh, poi_list, sqrt_num_of_box, geo_tree_node_id, highway_node_id_with_box_id_map,
                       all_highway_node, geo_node_in_partition_tree_unordered_map,
                       geopairs, distance_poi_to_highway_node_map, path_poi_to_highway_node_map,
                       source_poi_index, destination_poi_index, distance_result, path_result);
    distance_result = round(distance_result * 1000000000.0) / 1000000000.0;

    auto stop_query_time = std::chrono::high_resolution_clock::now();
    auto duration_query_time = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_query_time - start_query_time);
    query_time = duration_query_time.count();
    query_time /= 1000000;

    auto start_knn_query_time = std::chrono::high_resolution_clock::now();

    if (run_knn_query)
    {
        all_poi_knn_or_range_query_EAR_oracle_T(&mesh, poi_list, sqrt_num_of_box, geo_tree_node_id, highway_node_id_with_box_id_map,
                                                all_highway_node, geo_node_in_partition_tree_unordered_map,
                                                geopairs, distance_poi_to_highway_node_map, path_poi_to_highway_node_map,
                                                poi_num, 1, k_value, range, all_poi_knn_query_list);
    }

    auto stop_knn_query_time = std::chrono::high_resolution_clock::now();
    auto duration_knn_query_time = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_knn_query_time - start_knn_query_time);
    knn_query_time = duration_knn_query_time.count();
    knn_query_time /= 1000000;

    auto start_range_query_time = std::chrono::high_resolution_clock::now();

    if (run_range_query)
    {
        all_poi_knn_or_range_query_EAR_oracle_T(&mesh, poi_list, sqrt_num_of_box, geo_tree_node_id, highway_node_id_with_box_id_map,
                                                all_highway_node, geo_node_in_partition_tree_unordered_map,
                                                geopairs, distance_poi_to_highway_node_map, path_poi_to_highway_node_map,
                                                poi_num, 2, k_value, range, all_poi_knn_query_list);
    }

    auto stop_range_query_time = std::chrono::high_resolution_clock::now();
    auto duration_range_query_time = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_range_query_time - start_range_query_time);
    range_query_time = duration_range_query_time.count();
    range_query_time /= 1000000;
}

void FastFly(point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list,
             int source_poi_index, int destination_poi_index,
             double &query_time, double &memory_usage, double &distance_result,
             std::vector<point_cloud_geodesic::PathPoint> &path_result)
{
    auto start_query_time = std::chrono::high_resolution_clock::now();

    point_cloud_geodesic::PointCloudGeodesicAlgorithmDijkstra algorithm(point_cloud);
    double const distance_limit = point_cloud_geodesic::INFIN;
    point_cloud_geodesic::PathPoint source(&point_cloud->pc_points()[poi_list[source_poi_index]]);
    point_cloud_geodesic::PathPoint destination(&point_cloud->pc_points()[poi_list[destination_poi_index]]);
    std::vector<point_cloud_geodesic::PathPoint> one_source_poi_list(1, source);
    std::vector<point_cloud_geodesic::PathPoint> one_destination_poi_list(1, destination);
    algorithm.propagate(one_source_poi_list, distance_limit);
    algorithm.trace_back(destination, path_result);
    distance_result = length(path_result);
    distance_result = round(distance_result * 1000000000.0) / 1000000000.0;
    memory_usage += algorithm.get_memory() + path_result.size() * sizeof(point_cloud_geodesic::PathPoint) + sizeof(double);

    auto stop_query_time = std::chrono::high_resolution_clock::now();
    auto duration_query_time = std::chrono::duration_cast<std::chrono::microseconds>(stop_query_time - start_query_time);
    query_time = duration_query_time.count();
    query_time /= 1000;
}

void Kaul_Adapt_Dijk_Adapt(point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list, double epsilon,
                           bool pass_point_and_not_pass_terrain, int source_poi_index, int destination_poi_index,
                           double &point_cloud_to_terrain_time, double &query_time, double &point_cloud_to_terrain_memory_usage,
                           double &memory_usage, double &distance_result, std::vector<geodesic::SurfacePoint> &path_result)
{
    geodesic::Mesh mesh;
    point_cloud_to_terrain_and_initialize_terrain(point_cloud, &mesh, point_cloud_to_terrain_time, point_cloud_to_terrain_memory_usage);

    auto start_query_time = std::chrono::high_resolution_clock::now();

    double subdivision_level = 0;
    if (!pass_point_and_not_pass_terrain)
    {
        subdivision_level = epslion_to_subdivision_level(epsilon);
    }
    geodesic::GeodesicAlgorithmSubdivision algorithm(&mesh, subdivision_level);
    double const distance_limit = geodesic::GEODESIC_INF;
    geodesic::SurfacePoint source(&mesh.vertices()[poi_list[source_poi_index]]);
    geodesic::SurfacePoint destination(&mesh.vertices()[poi_list[destination_poi_index]]);
    std::vector<geodesic::SurfacePoint> one_source_poi_list(1, source);
    std::vector<geodesic::SurfacePoint> one_destination_poi_list(1, destination);
    algorithm.propagate(one_source_poi_list, distance_limit);
    algorithm.trace_back(destination, path_result);
    if (subdivision_level == 0)
    {
        modify_path(path_result);
    }
    distance_result = length(path_result);
    distance_result = round(distance_result * 1000000000.0) / 1000000000.0;
    memory_usage += algorithm.get_memory() + path_result.size() * sizeof(geodesic::SurfacePoint) + sizeof(double);

    auto stop_query_time = std::chrono::high_resolution_clock::now();
    auto duration_query_time = std::chrono::duration_cast<std::chrono::microseconds>(stop_query_time - start_query_time);
    query_time = duration_query_time.count();
    query_time /= 1000;
}

void CH_Adapt(point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list,
              int source_poi_index, int destination_poi_index, double &point_cloud_to_terrain_time,
              double &query_time, double &point_cloud_to_terrain_memory_usage, double &memory_usage,
              double &distance_result, std::vector<geodesic::SurfacePoint> &path_result)
{
    geodesic::Mesh mesh;
    point_cloud_to_terrain_and_initialize_terrain(point_cloud, &mesh, point_cloud_to_terrain_time, point_cloud_to_terrain_memory_usage);

    auto start_query_time = std::chrono::high_resolution_clock::now();

    geodesic::GeodesicAlgorithmExact algorithm(&mesh);
    double const distance_limit = geodesic::GEODESIC_INF;
    geodesic::SurfacePoint source(&mesh.vertices()[poi_list[source_poi_index]]);
    geodesic::SurfacePoint destination(&mesh.vertices()[poi_list[destination_poi_index]]);
    std::vector<geodesic::SurfacePoint> one_source_poi_list(1, source);
    std::vector<geodesic::SurfacePoint> one_destination_poi_list(1, destination);
    algorithm.propagate(one_source_poi_list, distance_limit);
    algorithm.trace_back(destination, path_result);
    distance_result = length(path_result);
    distance_result = round(distance_result * 1000000000.0) / 1000000000.0;
    memory_usage += algorithm.get_memory() + path_result.size() * sizeof(geodesic::SurfacePoint) + sizeof(double);

    auto stop_query_time = std::chrono::high_resolution_clock::now();
    auto duration_query_time = std::chrono::duration_cast<std::chrono::microseconds>(stop_query_time - start_query_time);
    query_time = duration_query_time.count();
    query_time /= 1000;
}

void FastFly_all_poi_knn_or_range_query(point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list,
                                        int knn_one_range_two, int k_value, double range,
                                        double &knn_or_range_query_time, std::vector<std::vector<int>> &all_poi_knn_or_range_list)
{
    auto start_knn_or_range_query_time = std::chrono::high_resolution_clock::now();

    std::vector<std::vector<std::pair<double, int>>> poi_to_other_poi_distance_and_index_list;
    poi_to_other_poi_distance_and_index_list.clear();
    std::vector<point_cloud_geodesic::PathPoint> one_source_poi_list;
    std::vector<std::pair<double, int>> one_poi_to_other_poi_distance_and_index_list;
    point_cloud_geodesic::PointCloudGeodesicAlgorithmDijkstra algorithm(point_cloud);
    double const distance_limit = point_cloud_geodesic::INFIN;
    for (int i = 0; i < poi_list.size(); i++)
    {
        one_source_poi_list.clear();
        one_poi_to_other_poi_distance_and_index_list.clear();
        one_source_poi_list.push_back(point_cloud_geodesic::PathPoint(&point_cloud->pc_points()[poi_list[i]]));
        algorithm.propagate(one_source_poi_list, distance_limit);
        for (int j = 0; j < poi_list.size(); j++)
        {
            double distance;
            point_cloud_geodesic::PathPoint one_destination_poi(&point_cloud->pc_points()[poi_list[j]]);
            algorithm.best_source(one_destination_poi, distance);
            one_poi_to_other_poi_distance_and_index_list.push_back(std::make_pair(distance, j));
        }
        std::sort(one_poi_to_other_poi_distance_and_index_list.begin(), one_poi_to_other_poi_distance_and_index_list.end());
        poi_to_other_poi_distance_and_index_list.push_back(one_poi_to_other_poi_distance_and_index_list);
    }
    knn_or_range_query(knn_one_range_two, k_value, range, poi_to_other_poi_distance_and_index_list, all_poi_knn_or_range_list);

    auto stop_knn_or_range_query_time = std::chrono::high_resolution_clock::now();
    auto duration_knn_or_range_query_time = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_knn_or_range_query_time - start_knn_or_range_query_time);
    knn_or_range_query_time = duration_knn_or_range_query_time.count();
    knn_or_range_query_time /= 1000000;
}

void Kaul_Adapt_Dijk_Adapt_all_poi_knn_or_range_query(point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list,
                                                      double epsilon, bool pass_point_and_not_pass_terrain,
                                                      int knn_one_range_two, int k_value, double range,
                                                      double &knn_or_range_query_time, std::vector<std::vector<int>> &all_poi_knn_or_range_list)
{
    double point_cloud_to_terrain_time = 0;
    double point_cloud_to_terrain_memory_usage = 0;
    geodesic::Mesh mesh;
    point_cloud_to_terrain_and_initialize_terrain(point_cloud, &mesh, point_cloud_to_terrain_time, point_cloud_to_terrain_memory_usage);

    auto start_knn_or_range_query_time = std::chrono::high_resolution_clock::now();

    std::vector<std::vector<std::pair<double, int>>> poi_to_other_poi_distance_and_index_list;
    poi_to_other_poi_distance_and_index_list.clear();
    std::vector<geodesic::SurfacePoint> one_source_poi_list;
    std::vector<std::pair<double, int>> one_poi_to_other_poi_distance_and_index_list;
    double subdivision_level = 0;
    if (!pass_point_and_not_pass_terrain)
    {
        subdivision_level = epslion_to_subdivision_level(epsilon);
    }
    geodesic::GeodesicAlgorithmSubdivision algorithm(&mesh, subdivision_level);
    double const distance_limit = geodesic::GEODESIC_INF;
    for (int i = 0; i < poi_list.size(); i++)
    {
        one_source_poi_list.clear();
        one_poi_to_other_poi_distance_and_index_list.clear();
        one_source_poi_list.push_back(geodesic::SurfacePoint(&mesh.vertices()[poi_list[i]]));
        algorithm.propagate(one_source_poi_list, distance_limit);
        for (int j = 0; j < poi_list.size(); j++)
        {
            double distance;
            geodesic::SurfacePoint one_destination_poi(&mesh.vertices()[poi_list[j]]);
            algorithm.best_source(one_destination_poi, distance);
            one_poi_to_other_poi_distance_and_index_list.push_back(std::make_pair(distance, j));
        }
        std::sort(one_poi_to_other_poi_distance_and_index_list.begin(), one_poi_to_other_poi_distance_and_index_list.end());
        poi_to_other_poi_distance_and_index_list.push_back(one_poi_to_other_poi_distance_and_index_list);
    }
    knn_or_range_query(knn_one_range_two, k_value, range, poi_to_other_poi_distance_and_index_list, all_poi_knn_or_range_list);

    auto stop_knn_or_range_query_time = std::chrono::high_resolution_clock::now();
    auto duration_knn_or_range_query_time = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_knn_or_range_query_time - start_knn_or_range_query_time);
    knn_or_range_query_time = duration_knn_or_range_query_time.count();
    knn_or_range_query_time /= 1000000;
}

void CH_Adapt_all_poi_knn_or_range_query(point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list,
                                         int knn_one_range_two, int k_value, double range,
                                         double &knn_or_range_query_time, std::vector<std::vector<int>> &all_poi_knn_or_range_list)
{
    double point_cloud_to_terrain_time = 0;
    double point_cloud_to_terrain_memory_usage = 0;
    geodesic::Mesh mesh;
    point_cloud_to_terrain_and_initialize_terrain(point_cloud, &mesh, point_cloud_to_terrain_time, point_cloud_to_terrain_memory_usage);

    auto start_knn_or_range_query_time = std::chrono::high_resolution_clock::now();

    std::vector<std::vector<std::pair<double, int>>> poi_to_other_poi_distance_and_index_list;
    poi_to_other_poi_distance_and_index_list.clear();
    std::vector<geodesic::SurfacePoint> one_source_poi_list;
    std::vector<std::pair<double, int>> one_poi_to_other_poi_distance_and_index_list;
    geodesic::GeodesicAlgorithmExact algorithm(&mesh);
    double const distance_limit = geodesic::GEODESIC_INF;
    for (int i = 0; i < poi_list.size(); i++)
    {
        one_source_poi_list.clear();
        one_poi_to_other_poi_distance_and_index_list.clear();
        one_source_poi_list.push_back(geodesic::SurfacePoint(&mesh.vertices()[poi_list[i]]));
        algorithm.propagate(one_source_poi_list, distance_limit);
        for (int j = 0; j < poi_list.size(); j++)
        {
            double distance;
            geodesic::SurfacePoint one_destination_poi(&mesh.vertices()[poi_list[j]]);
            algorithm.best_source(one_destination_poi, distance);
            one_poi_to_other_poi_distance_and_index_list.push_back(std::make_pair(distance, j));
        }
        std::sort(one_poi_to_other_poi_distance_and_index_list.begin(), one_poi_to_other_poi_distance_and_index_list.end());
        poi_to_other_poi_distance_and_index_list.push_back(one_poi_to_other_poi_distance_and_index_list);
    }
    knn_or_range_query(knn_one_range_two, k_value, range, poi_to_other_poi_distance_and_index_list, all_poi_knn_or_range_list);

    auto stop_knn_or_range_query_time = std::chrono::high_resolution_clock::now();
    auto duration_knn_or_range_query_time = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_knn_or_range_query_time - start_knn_or_range_query_time);
    knn_or_range_query_time = duration_knn_or_range_query_time.count();
    knn_or_range_query_time /= 1000000;
}

void RC_Oracle_with_output(int poi_num, point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list, double epsilon,
                           int source_poi_index, int destination_poi_index, double point_cloud_exact_distance,
                           double terrain_exact_distance, bool run_knn_query, bool run_range_query,
                           int k_value, double range,
                           std::vector<std::vector<int>> &point_cloud_exact_all_poi_knn_query_list,
                           std::vector<std::vector<int>> &terrain_exact_all_poi_knn_query_list,
                           std::vector<std::vector<int>> &point_cloud_exact_all_poi_range_query_list,
                           std::vector<std::vector<int>> &terrain_exact_all_poi_range_query_list, std::string write_file_header)
{
    std::unordered_map<int, double> distance_poi_to_poi_map;
    std::unordered_map<int, std::vector<point_cloud_geodesic::PathPoint>> path_poi_to_poi_map;
    std::unordered_map<int, int> non_exact_source_poi_map;
    std::unordered_map<int, int> exact_source_poi_process_order_map;

    double construction_time = 0;
    double query_time = 0;
    double knn_query_time = 0;
    double range_query_time = 0;
    double memory_usage = 0;
    double index_size = 0;
    double distance_result = 0;
    double point_cloud_knn_query_error = 0;
    double terrain_knn_query_error = 0;
    double point_cloud_range_query_error = 0;
    double terrain_range_query_error = 0;
    std::vector<point_cloud_geodesic::PathPoint> path_result;
    std::vector<std::vector<int>> all_poi_knn_query_list;
    std::vector<std::vector<int>> all_poi_range_query_list;
    path_result.clear();
    all_poi_knn_query_list.clear();
    all_poi_range_query_list.clear();

    RC_Oracle(poi_num, point_cloud, poi_list, epsilon, distance_poi_to_poi_map, path_poi_to_poi_map,
              non_exact_source_poi_map, exact_source_poi_process_order_map, construction_time, memory_usage, index_size);
    RC_Oracle_query(poi_num, distance_poi_to_poi_map, path_poi_to_poi_map, non_exact_source_poi_map, exact_source_poi_process_order_map,
                    source_poi_index, destination_poi_index, distance_result, path_result, query_time);
    if (run_knn_query)
    {
        RC_Oracle_all_poi_knn_or_range_query(poi_num, distance_poi_to_poi_map, non_exact_source_poi_map, exact_source_poi_process_order_map, 1, k_value, range, knn_query_time, all_poi_knn_query_list);
        calculate_knn_or_range_query_error(point_cloud_exact_all_poi_knn_query_list, all_poi_knn_query_list, point_cloud_knn_query_error);
        calculate_knn_or_range_query_error(terrain_exact_all_poi_knn_query_list, all_poi_knn_query_list, terrain_knn_query_error);
    }
    if (run_range_query)
    {
        RC_Oracle_all_poi_knn_or_range_query(poi_num, distance_poi_to_poi_map, non_exact_source_poi_map, exact_source_poi_process_order_map, 2, k_value, range, range_query_time, all_poi_range_query_list);
        calculate_knn_or_range_query_error(point_cloud_exact_all_poi_range_query_list, all_poi_range_query_list, point_cloud_range_query_error);
        calculate_knn_or_range_query_error(terrain_exact_all_poi_range_query_list, all_poi_range_query_list, terrain_range_query_error);
    }

    std::cout << "Preprocessing time: " << construction_time << " ms" << std::endl;
    std::cout << "Query time: " << query_time << " ms" << std::endl;
    std::cout << "Memory usage: " << memory_usage / 1e6 << " MB" << std::endl;
    std::cout << "Index size: " << index_size / 1e6 << " MB" << std::endl;
    std::cout << "Calculated distance: " << distance_result << ", point cloud exact distance: " << point_cloud_exact_distance << ", point cloud distance error: " << distance_result / point_cloud_exact_distance - 1 << ", terrain exact distance: " << terrain_exact_distance << ", terrain distance error: " << distance_result / terrain_exact_distance - 1 << std::endl;
    if (run_knn_query)
    {
        std::cout << "Knn query time: " << knn_query_time << " ms" << std::endl;
        std::cout << "Point cloud knn error: " << point_cloud_knn_query_error << ", terrain knn error: " << terrain_knn_query_error << std::endl;
    }
    if (run_range_query)
    {
        std::cout << "Range query time: " << range_query_time << " ms" << std::endl;
        std::cout << "Point cloud range error: " << point_cloud_range_query_error << ", terrain range error: " << terrain_range_query_error << std::endl;
    }

    std::ofstream ofs("../output/output.txt", std::ios_base::app);
    ofs << "== RC_Oracle ==\n";
    ofs << write_file_header << "\t"
        << 0 << "\t"
        << construction_time << "\t"
        << query_time << "\t"
        << 0 << "\t"
        << memory_usage / 1e6 << "\t"
        << index_size / 1e6 << "\t"
        << distance_result / point_cloud_exact_distance - 1 << "\t"
        << distance_result / terrain_exact_distance - 1 << "\t"
        << knn_query_time << "\t"
        << point_cloud_knn_query_error << "\t"
        << terrain_knn_query_error << "\t"
        << range_query_time << "\t"
        << point_cloud_range_query_error << "\t"
        << terrain_range_query_error << "\n\n";
    ofs.close();
}

void RC_Oracle_Naive_with_output(int poi_num, point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list,
                                 int source_poi_index, int destination_poi_index, double point_cloud_exact_distance,
                                 double terrain_exact_distance, bool run_knn_query, bool run_range_query,
                                 int k_value, double range,
                                 std::vector<std::vector<int>> &point_cloud_exact_all_poi_knn_query_list,
                                 std::vector<std::vector<int>> &terrain_exact_all_poi_knn_query_list,
                                 std::vector<std::vector<int>> &point_cloud_exact_all_poi_range_query_list,
                                 std::vector<std::vector<int>> &terrain_exact_all_poi_range_query_list, std::string write_file_header)
{
    std::unordered_map<int, double> pairwise_distance_poi_to_poi_map;
    std::unordered_map<int, std::vector<point_cloud_geodesic::PathPoint>> pairwise_path_poi_to_poi_map;

    double construction_time = 0;
    double query_time = 0;
    double knn_query_time = 0;
    double range_query_time = 0;
    double memory_usage = 0;
    double index_size = 0;
    double distance_result = 0;
    double point_cloud_knn_query_error = 0;
    double terrain_knn_query_error = 0;
    double point_cloud_range_query_error = 0;
    double terrain_range_query_error = 0;
    std::vector<point_cloud_geodesic::PathPoint> path_result;
    std::vector<std::vector<int>> all_poi_knn_query_list;
    std::vector<std::vector<int>> all_poi_range_query_list;
    path_result.clear();
    all_poi_knn_query_list.clear();
    all_poi_range_query_list.clear();

    RC_Oracle_Naive(poi_num, point_cloud, poi_list, pairwise_distance_poi_to_poi_map, pairwise_path_poi_to_poi_map,
                    construction_time, memory_usage, index_size);
    RC_Oracle_Naive_query(poi_num, pairwise_distance_poi_to_poi_map, pairwise_path_poi_to_poi_map,
                          source_poi_index, destination_poi_index, distance_result, path_result, query_time);
    if (run_knn_query)
    {
        RC_Oracle_Naive_all_poi_knn_or_range_query(poi_num, pairwise_distance_poi_to_poi_map, 1, k_value, range, knn_query_time, all_poi_knn_query_list);
        calculate_knn_or_range_query_error(point_cloud_exact_all_poi_knn_query_list, all_poi_knn_query_list, point_cloud_knn_query_error);
        calculate_knn_or_range_query_error(terrain_exact_all_poi_knn_query_list, all_poi_knn_query_list, terrain_knn_query_error);
    }
    if (run_range_query)
    {
        RC_Oracle_Naive_all_poi_knn_or_range_query(poi_num, pairwise_distance_poi_to_poi_map, 2, k_value, range, range_query_time, all_poi_range_query_list);
        calculate_knn_or_range_query_error(point_cloud_exact_all_poi_range_query_list, all_poi_range_query_list, point_cloud_range_query_error);
        calculate_knn_or_range_query_error(terrain_exact_all_poi_range_query_list, all_poi_range_query_list, terrain_range_query_error);
    }

    std::cout << "Preprocessing time: " << construction_time << " ms" << std::endl;
    std::cout << "Query time: " << query_time << " ms" << std::endl;
    std::cout << "Memory usage: " << memory_usage / 1e6 << " MB" << std::endl;
    std::cout << "Index size: " << index_size / 1e6 << " MB" << std::endl;
    std::cout << "Calculated distance: " << distance_result << ", point cloud exact distance: " << point_cloud_exact_distance << ", point cloud distance error: " << distance_result / point_cloud_exact_distance - 1 << ", terrain exact distance: " << terrain_exact_distance << ", terrain distance error: " << distance_result / terrain_exact_distance - 1 << std::endl;
    if (run_knn_query)
    {
        std::cout << "Knn query time: " << knn_query_time << " ms" << std::endl;
        std::cout << "Point cloud knn error: " << point_cloud_knn_query_error << ", terrain knn error: " << terrain_knn_query_error << std::endl;
    }
    if (run_range_query)
    {
        std::cout << "Range query time: " << range_query_time << " ms" << std::endl;
        std::cout << "Point cloud range error: " << point_cloud_range_query_error << ", terrain range error: " << terrain_range_query_error << std::endl;
    }

    std::ofstream ofs("../output/output.txt", std::ios_base::app);
    ofs << "== RC_Oracle_Naive ==\n";
    ofs << write_file_header << "\t"
        << 0 << "\t"
        << construction_time << "\t"
        << query_time << "\t"
        << 0 << "\t"
        << memory_usage / 1e6 << "\t"
        << index_size / 1e6 << "\t"
        << distance_result / point_cloud_exact_distance - 1 << "\t"
        << distance_result / terrain_exact_distance - 1 << "\t"
        << knn_query_time << "\t"
        << point_cloud_knn_query_error << "\t"
        << terrain_knn_query_error << "\t"
        << range_query_time << "\t"
        << point_cloud_range_query_error << "\t"
        << terrain_range_query_error << "\n\n";
    ofs.close();
}

void SE_Oracle_FastFly_Adapt_with_output(int poi_num, point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list, double epsilon,
                                         int source_poi_index, int destination_poi_index, double point_cloud_exact_distance,
                                         double terrain_exact_distance, bool run_knn_query, bool run_range_query,
                                         int k_value, double range,
                                         std::vector<std::vector<int>> &point_cloud_exact_all_poi_knn_query_list,
                                         std::vector<std::vector<int>> &terrain_exact_all_poi_knn_query_list,
                                         std::vector<std::vector<int>> &point_cloud_exact_all_poi_range_query_list,
                                         std::vector<std::vector<int>> &terrain_exact_all_poi_range_query_list, std::string write_file_header)
{
    double construction_time = 0;
    double query_time = 0;
    double knn_query_time = 0;
    double range_query_time = 0;
    double memory_usage = 0;
    double index_size = 0;
    double distance_result = 0;
    double point_cloud_knn_query_error = 0;
    double terrain_knn_query_error = 0;
    double point_cloud_range_query_error = 0;
    double terrain_range_query_error = 0;
    std::vector<point_cloud_geodesic::PathPoint> path_result;
    std::vector<std::vector<int>> all_poi_knn_query_list;
    std::vector<std::vector<int>> all_poi_range_query_list;
    path_result.clear();
    all_poi_knn_query_list.clear();
    all_poi_range_query_list.clear();

    SE_Oracle_FastFly_Adapt(poi_num, point_cloud, poi_list, epsilon, source_poi_index, destination_poi_index, construction_time,
                            query_time, memory_usage, index_size, distance_result, path_result, run_knn_query, run_range_query,
                            k_value, range, knn_query_time, all_poi_knn_query_list, range_query_time, all_poi_range_query_list);
    if (run_knn_query)
    {
        calculate_knn_or_range_query_error(point_cloud_exact_all_poi_knn_query_list, all_poi_knn_query_list, point_cloud_knn_query_error);
        calculate_knn_or_range_query_error(terrain_exact_all_poi_knn_query_list, all_poi_knn_query_list, terrain_knn_query_error);
    }
    if (run_range_query)
    {
        calculate_knn_or_range_query_error(point_cloud_exact_all_poi_range_query_list, all_poi_range_query_list, point_cloud_range_query_error);
        calculate_knn_or_range_query_error(terrain_exact_all_poi_range_query_list, all_poi_range_query_list, terrain_range_query_error);
    }

    std::cout << "Preprocessing time: " << construction_time << " ms" << std::endl;
    std::cout << "Query time: " << query_time << " ms" << std::endl;
    std::cout << "Memory usage: " << memory_usage / 1e6 << " MB" << std::endl;
    std::cout << "Index size: " << index_size / 1e6 << " MB" << std::endl;
    std::cout << "Calculated distance: " << distance_result << ", point cloud exact distance: " << point_cloud_exact_distance << ", point cloud distance error: " << distance_result / point_cloud_exact_distance - 1 << ", terrain exact distance: " << terrain_exact_distance << ", terrain distance error: " << distance_result / terrain_exact_distance - 1 << std::endl;
    if (run_knn_query)
    {
        std::cout << "Knn query time: " << knn_query_time << " ms" << std::endl;
        std::cout << "Point cloud knn error: " << point_cloud_knn_query_error << ", terrain knn error: " << terrain_knn_query_error << std::endl;
    }
    if (run_range_query)
    {
        std::cout << "Range query time: " << range_query_time << " ms" << std::endl;
        std::cout << "Point cloud range error: " << point_cloud_range_query_error << ", terrain range error: " << terrain_range_query_error << std::endl;
    }

    std::ofstream ofs("../output/output.txt", std::ios_base::app);
    ofs << "== SE_Oracle_FastFly_Adapt ==\n";
    ofs << write_file_header << "\t"
        << 0 << "\t"
        << construction_time << "\t"
        << query_time << "\t"
        << 0 << "\t"
        << memory_usage / 1e6 << "\t"
        << index_size / 1e6 << "\t"
        << distance_result / point_cloud_exact_distance - 1 << "\t"
        << distance_result / terrain_exact_distance - 1 << "\t"
        << knn_query_time << "\t"
        << point_cloud_knn_query_error << "\t"
        << terrain_knn_query_error << "\t"
        << range_query_time << "\t"
        << point_cloud_range_query_error << "\t"
        << terrain_range_query_error << "\n\n";
    ofs.close();
}

void SE_Oracle_Adapt_with_output(int poi_num, point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list, double epsilon,
                                 int source_poi_index, int destination_poi_index, double point_cloud_exact_distance,
                                 double terrain_exact_distance, bool run_knn_query, bool run_range_query,
                                 int k_value, double range,
                                 std::vector<std::vector<int>> &point_cloud_exact_all_poi_knn_query_list,
                                 std::vector<std::vector<int>> &terrain_exact_all_poi_knn_query_list,
                                 std::vector<std::vector<int>> &point_cloud_exact_all_poi_range_query_list,
                                 std::vector<std::vector<int>> &terrain_exact_all_poi_range_query_list, std::string write_file_header)
{
    double point_cloud_to_terrain_time = 0;
    double construction_time = 0;
    double query_time = 0;
    double knn_query_time = 0;
    double range_query_time = 0;
    double point_cloud_to_terrain_memory_usage = 0;
    double memory_usage = 0;
    double index_size = 0;
    double distance_result = 0;
    double point_cloud_knn_query_error = 0;
    double terrain_knn_query_error = 0;
    double point_cloud_range_query_error = 0;
    double terrain_range_query_error = 0;
    std::vector<geodesic::SurfacePoint> path_result;
    std::vector<std::vector<int>> all_poi_knn_query_list;
    std::vector<std::vector<int>> all_poi_range_query_list;
    path_result.clear();
    all_poi_knn_query_list.clear();
    all_poi_range_query_list.clear();

    SE_Oracle_Adapt(
        poi_num, point_cloud, poi_list, epsilon, source_poi_index, destination_poi_index,
        point_cloud_to_terrain_time, construction_time, query_time, point_cloud_to_terrain_memory_usage,
        memory_usage, index_size, distance_result, path_result, run_knn_query, run_range_query,
        k_value, range, knn_query_time, all_poi_knn_query_list, range_query_time, all_poi_range_query_list);
    if (run_knn_query)
    {
        calculate_knn_or_range_query_error(point_cloud_exact_all_poi_knn_query_list, all_poi_knn_query_list, point_cloud_knn_query_error);
        calculate_knn_or_range_query_error(terrain_exact_all_poi_knn_query_list, all_poi_knn_query_list, terrain_knn_query_error);
    }
    if (run_range_query)
    {
        calculate_knn_or_range_query_error(point_cloud_exact_all_poi_range_query_list, all_poi_range_query_list, point_cloud_range_query_error);
        calculate_knn_or_range_query_error(terrain_exact_all_poi_range_query_list, all_poi_range_query_list, terrain_range_query_error);
    }

    std::cout << "Point cloud to terrain time: " << point_cloud_to_terrain_time << " ms" << std::endl;
    std::cout << "Preprocessing time: " << construction_time << " ms" << std::endl;
    std::cout << "Query time: " << query_time << " ms" << std::endl;
    std::cout << "Point cloud to terrain memory usage: " << point_cloud_to_terrain_memory_usage / 1e6 << " MB" << std::endl;
    std::cout << "Memory usage: " << memory_usage / 1e6 << " MB" << std::endl;
    std::cout << "Index size: " << index_size / 1e6 << " MB" << std::endl;
    std::cout << "Calculated distance: " << distance_result << ", point cloud exact distance: " << point_cloud_exact_distance << ", point cloud distance error: " << distance_result / point_cloud_exact_distance - 1 << ", terrain exact distance: " << terrain_exact_distance << ", terrain distance error: " << distance_result / terrain_exact_distance - 1 << std::endl;
    if (run_knn_query)
    {
        std::cout << "Knn query time: " << knn_query_time << " ms" << std::endl;
        std::cout << "Point cloud knn error: " << point_cloud_knn_query_error << ", terrain knn error: " << terrain_knn_query_error << std::endl;
    }
    if (run_range_query)
    {
        std::cout << "Range query time: " << range_query_time << " ms" << std::endl;
        std::cout << "Point cloud range error: " << point_cloud_range_query_error << ", terrain range error: " << terrain_range_query_error << std::endl;
    }

    std::ofstream ofs("../output/output.txt", std::ios_base::app);
    ofs << "== SE_Oracle_Adapt ==\n";
    ofs << write_file_header << "\t"
        << point_cloud_to_terrain_time << "\t"
        << construction_time << "\t"
        << query_time << "\t"
        << point_cloud_to_terrain_memory_usage / 1e6 << "\t"
        << memory_usage / 1e6 << "\t"
        << index_size / 1e6 << "\t"
        << distance_result / point_cloud_exact_distance - 1 << "\t"
        << distance_result / terrain_exact_distance - 1 << "\t"
        << knn_query_time << "\t"
        << point_cloud_knn_query_error << "\t"
        << terrain_knn_query_error << "\t"
        << range_query_time << "\t"
        << point_cloud_range_query_error << "\t"
        << terrain_range_query_error << "\n\n";
    ofs.close();
}

void EAR_Oracle_FastFly_Adapt_with_output(int poi_num, point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list, double epsilon,
                                          int source_poi_index, int destination_poi_index, double point_cloud_exact_distance,
                                          double terrain_exact_distance, bool run_knn_query, bool run_range_query,
                                          int k_value, double range,
                                          std::vector<std::vector<int>> &point_cloud_exact_all_poi_knn_query_list,
                                          std::vector<std::vector<int>> &terrain_exact_all_poi_knn_query_list,
                                          std::vector<std::vector<int>> &point_cloud_exact_all_poi_range_query_list,
                                          std::vector<std::vector<int>> &terrain_exact_all_poi_range_query_list, std::string write_file_header)
{
    int sqrt_num_of_box = 2;
    double construction_time = 0;
    double query_time = 0;
    double knn_query_time = 0;
    double range_query_time = 0;
    double memory_usage = 0;
    double index_size = 0;
    double distance_result = 0;
    double point_cloud_knn_query_error = 0;
    double terrain_knn_query_error = 0;
    double point_cloud_range_query_error = 0;
    double terrain_range_query_error = 0;
    std::vector<point_cloud_geodesic::PathPoint> path_result;
    std::vector<std::vector<int>> all_poi_knn_query_list;
    std::vector<std::vector<int>> all_poi_range_query_list;
    path_result.clear();
    all_poi_knn_query_list.clear();
    all_poi_range_query_list.clear();

    EAR_Oracle_FastFly_Adapt(
        sqrt_num_of_box, poi_num, point_cloud, poi_list, epsilon, source_poi_index, destination_poi_index,
        construction_time, query_time, memory_usage, index_size, distance_result, path_result, run_knn_query, run_range_query,
        k_value, range, knn_query_time, all_poi_knn_query_list, range_query_time, all_poi_range_query_list);
    if (run_knn_query)
    {
        calculate_knn_or_range_query_error(point_cloud_exact_all_poi_knn_query_list, all_poi_knn_query_list, point_cloud_knn_query_error);
        calculate_knn_or_range_query_error(terrain_exact_all_poi_knn_query_list, all_poi_knn_query_list, terrain_knn_query_error);
    }
    if (run_range_query)
    {
        calculate_knn_or_range_query_error(point_cloud_exact_all_poi_range_query_list, all_poi_range_query_list, point_cloud_range_query_error);
        calculate_knn_or_range_query_error(terrain_exact_all_poi_range_query_list, all_poi_range_query_list, terrain_range_query_error);
    }

    std::cout << "Preprocessing time: " << construction_time << " ms" << std::endl;
    std::cout << "Query time: " << query_time << " ms" << std::endl;
    std::cout << "Memory usage: " << memory_usage / 1e6 << " MB" << std::endl;
    std::cout << "Index size: " << index_size / 1e6 << " MB" << std::endl;
    std::cout << "Calculated distance: " << distance_result << ", point cloud exact distance: " << point_cloud_exact_distance << ", point cloud distance error: " << distance_result / point_cloud_exact_distance - 1 << ", terrain exact distance: " << terrain_exact_distance << ", terrain distance error: " << distance_result / terrain_exact_distance - 1 << std::endl;
    if (run_knn_query)
    {
        std::cout << "Knn query time: " << knn_query_time << " ms" << std::endl;
        std::cout << "Point cloud knn error: " << point_cloud_knn_query_error << ", terrain knn error: " << terrain_knn_query_error << std::endl;
    }
    if (run_range_query)
    {
        std::cout << "Range query time: " << range_query_time << " ms" << std::endl;
        std::cout << "Point cloud range error: " << point_cloud_range_query_error << ", terrain range error: " << terrain_range_query_error << std::endl;
    }

    std::ofstream ofs("../output/output.txt", std::ios_base::app);
    ofs << "== EAR_Oracle_FastFly_Adapt ==\n";
    ofs << write_file_header << "\t"
        << 0 << "\t"
        << construction_time << "\t"
        << query_time << "\t"
        << 0 << "\t"
        << memory_usage / 1e6 << "\t"
        << index_size / 1e6 << "\t"
        << distance_result / point_cloud_exact_distance - 1 << "\t"
        << distance_result / terrain_exact_distance - 1 << "\t"
        << knn_query_time << "\t"
        << point_cloud_knn_query_error << "\t"
        << terrain_knn_query_error << "\t"
        << range_query_time << "\t"
        << point_cloud_range_query_error << "\t"
        << terrain_range_query_error << "\n\n";
    ofs.close();
}

void EAR_Oracle_Adapt_with_output(int poi_num, point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list, double epsilon,
                                  int source_poi_index, int destination_poi_index, double point_cloud_exact_distance,
                                  double terrain_exact_distance, bool run_knn_query, bool run_range_query,
                                  int k_value, double range,
                                  std::vector<std::vector<int>> &point_cloud_exact_all_poi_knn_query_list,
                                  std::vector<std::vector<int>> &terrain_exact_all_poi_knn_query_list,
                                  std::vector<std::vector<int>> &point_cloud_exact_all_poi_range_query_list,
                                  std::vector<std::vector<int>> &terrain_exact_all_poi_range_query_list, std::string write_file_header)
{
    int sqrt_num_of_box = 2;
    double point_cloud_to_terrain_time = 0;
    double construction_time = 0;
    double query_time = 0;
    double knn_query_time = 0;
    double range_query_time = 0;
    double point_cloud_to_terrain_memory_usage = 0;
    double memory_usage = 0;
    double index_size = 0;
    double distance_result = 0;
    double point_cloud_knn_query_error = 0;
    double terrain_knn_query_error = 0;
    double point_cloud_range_query_error = 0;
    double terrain_range_query_error = 0;
    std::vector<geodesic::SurfacePoint> path_result;
    std::vector<std::vector<int>> all_poi_knn_query_list;
    std::vector<std::vector<int>> all_poi_range_query_list;
    path_result.clear();
    all_poi_knn_query_list.clear();
    all_poi_range_query_list.clear();

    EAR_Oracle_Adapt(
        sqrt_num_of_box, poi_num, point_cloud, poi_list, epsilon, source_poi_index, destination_poi_index,
        point_cloud_to_terrain_time, construction_time, query_time, point_cloud_to_terrain_memory_usage,
        memory_usage, index_size, distance_result, path_result, run_knn_query, run_range_query,
        k_value, range, knn_query_time, all_poi_knn_query_list, range_query_time, all_poi_range_query_list);
    if (run_knn_query)
    {
        calculate_knn_or_range_query_error(point_cloud_exact_all_poi_knn_query_list, all_poi_knn_query_list, point_cloud_knn_query_error);
        calculate_knn_or_range_query_error(terrain_exact_all_poi_knn_query_list, all_poi_knn_query_list, terrain_knn_query_error);
    }
    if (run_range_query)
    {
        calculate_knn_or_range_query_error(point_cloud_exact_all_poi_range_query_list, all_poi_range_query_list, point_cloud_range_query_error);
        calculate_knn_or_range_query_error(terrain_exact_all_poi_range_query_list, all_poi_range_query_list, terrain_range_query_error);
    }

    std::cout << "Point cloud to terrain time: " << point_cloud_to_terrain_time << " ms" << std::endl;
    std::cout << "Preprocessing time: " << construction_time << " ms" << std::endl;
    std::cout << "Query time: " << query_time << " ms" << std::endl;
    std::cout << "Point cloud to terrain memory usage: " << point_cloud_to_terrain_memory_usage / 1e6 << " MB" << std::endl;
    std::cout << "Memory usage: " << memory_usage / 1e6 << " MB" << std::endl;
    std::cout << "Index size: " << index_size / 1e6 << " MB" << std::endl;
    std::cout << "Calculated distance: " << distance_result << ", point cloud exact distance: " << point_cloud_exact_distance << ", point cloud distance error: " << distance_result / point_cloud_exact_distance - 1 << ", terrain exact distance: " << terrain_exact_distance << ", terrain distance error: " << distance_result / terrain_exact_distance - 1 << std::endl;
    if (run_knn_query)
    {
        std::cout << "Knn query time: " << knn_query_time << " ms" << std::endl;
        std::cout << "Point cloud knn error: " << point_cloud_knn_query_error << ", terrain knn error: " << terrain_knn_query_error << std::endl;
    }
    if (run_range_query)
    {
        std::cout << "Range query time: " << range_query_time << " ms" << std::endl;
        std::cout << "Point cloud range error: " << point_cloud_range_query_error << ", terrain range error: " << terrain_range_query_error << std::endl;
    }

    std::ofstream ofs("../output/output.txt", std::ios_base::app);
    ofs << "== EAR_Oracle_Adapt ==\n";
    ofs << write_file_header << "\t"
        << point_cloud_to_terrain_time << "\t"
        << construction_time << "\t"
        << query_time << "\t"
        << point_cloud_to_terrain_memory_usage / 1e6 << "\t"
        << memory_usage / 1e6 << "\t"
        << index_size / 1e6 << "\t"
        << distance_result / point_cloud_exact_distance - 1 << "\t"
        << distance_result / terrain_exact_distance - 1 << "\t"
        << knn_query_time << "\t"
        << point_cloud_knn_query_error << "\t"
        << terrain_knn_query_error << "\t"
        << range_query_time << "\t"
        << point_cloud_range_query_error << "\t"
        << terrain_range_query_error << "\n\n";
    ofs.close();
}

void FastFly_with_output(point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list,
                         int source_poi_index, int destination_poi_index, double point_cloud_exact_distance,
                         double terrain_exact_distance, bool run_knn_query, bool run_range_query,
                         int k_value, double range,
                         std::vector<std::vector<int>> &point_cloud_exact_all_poi_knn_query_list,
                         std::vector<std::vector<int>> &terrain_exact_all_poi_knn_query_list,
                         std::vector<std::vector<int>> &point_cloud_exact_all_poi_range_query_list,
                         std::vector<std::vector<int>> &terrain_exact_all_poi_range_query_list, std::string write_file_header)
{
    double query_time = 0;
    double knn_query_time = 0;
    double range_query_time = 0;
    double memory_usage = 0;
    double distance_result = 0;
    double point_cloud_knn_query_error = 0;
    double terrain_knn_query_error = 0;
    double point_cloud_range_query_error = 0;
    double terrain_range_query_error = 0;
    std::vector<point_cloud_geodesic::PathPoint> path_result;
    std::vector<std::vector<int>> all_poi_knn_query_list;
    std::vector<std::vector<int>> all_poi_range_query_list;
    path_result.clear();
    all_poi_knn_query_list.clear();
    all_poi_range_query_list.clear();

    FastFly(point_cloud, poi_list, source_poi_index, destination_poi_index,
            query_time, memory_usage, distance_result, path_result);
    if (run_knn_query)
    {
        FastFly_all_poi_knn_or_range_query(point_cloud, poi_list, 1, k_value, range, knn_query_time, all_poi_knn_query_list);
        calculate_knn_or_range_query_error(point_cloud_exact_all_poi_knn_query_list, all_poi_knn_query_list, point_cloud_knn_query_error);
        calculate_knn_or_range_query_error(terrain_exact_all_poi_knn_query_list, all_poi_knn_query_list, terrain_knn_query_error);
    }
    if (run_range_query)
    {
        FastFly_all_poi_knn_or_range_query(point_cloud, poi_list, 2, k_value, range, range_query_time, all_poi_range_query_list);
        calculate_knn_or_range_query_error(point_cloud_exact_all_poi_range_query_list, all_poi_range_query_list, point_cloud_range_query_error);
        calculate_knn_or_range_query_error(terrain_exact_all_poi_range_query_list, all_poi_range_query_list, terrain_range_query_error);
    }

    std::cout << "Query time: " << query_time << " ms" << std::endl;
    std::cout << "Memory usage: " << memory_usage / 1e6 << " MB" << std::endl;
    std::cout << "Calculated distance: " << distance_result << ", point cloud exact distance: " << point_cloud_exact_distance << ", point cloud distance error: " << distance_result / point_cloud_exact_distance - 1 << ", terrain exact distance: " << terrain_exact_distance << ", terrain distance error: " << distance_result / terrain_exact_distance - 1 << std::endl;
    if (run_knn_query)
    {
        std::cout << "Knn query time: " << knn_query_time << " ms" << std::endl;
        std::cout << "Point cloud knn error: " << point_cloud_knn_query_error << ", terrain knn error: " << terrain_knn_query_error << std::endl;
    }
    if (run_range_query)
    {
        std::cout << "Range query time: " << range_query_time << " ms" << std::endl;
        std::cout << "Point cloud range error: " << point_cloud_range_query_error << ", terrain range error: " << terrain_range_query_error << std::endl;
    }

    std::ofstream ofs("../output/output.txt", std::ios_base::app);
    ofs << "== FastFly ==\n";
    ofs << write_file_header << "\t"
        << 0 << "\t"
        << 0 << "\t"
        << query_time << "\t"
        << 0 << "\t"
        << memory_usage / 1e6 << "\t"
        << 0 << "\t"
        << distance_result / point_cloud_exact_distance - 1 << "\t"
        << distance_result / terrain_exact_distance - 1 << "\t"
        << knn_query_time << "\t"
        << point_cloud_knn_query_error << "\t"
        << terrain_knn_query_error << "\t"
        << range_query_time << "\t"
        << point_cloud_range_query_error << "\t"
        << terrain_range_query_error << "\n\n";
    ofs.close();
}

void Dijk_Adapt_with_output(point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list,
                            int source_poi_index, int destination_poi_index, double point_cloud_exact_distance,
                            double terrain_exact_distance, bool run_knn_query, bool run_range_query,
                            int k_value, double range,
                            std::vector<std::vector<int>> &point_cloud_exact_all_poi_knn_query_list,
                            std::vector<std::vector<int>> &terrain_exact_all_poi_knn_query_list,
                            std::vector<std::vector<int>> &point_cloud_exact_all_poi_range_query_list,
                            std::vector<std::vector<int>> &terrain_exact_all_poi_range_query_list, std::string write_file_header)
{
    double point_cloud_to_terrain_time = 0;
    double query_time = 0;
    double knn_query_time = 0;
    double range_query_time = 0;
    double point_cloud_to_terrain_memory_usage = 0;
    double memory_usage = 0;
    double distance_result = 0;
    double point_cloud_knn_query_error = 0;
    double terrain_knn_query_error = 0;
    double point_cloud_range_query_error = 0;
    double terrain_range_query_error = 0;
    std::vector<geodesic::SurfacePoint> path_result;
    std::vector<std::vector<int>> all_poi_knn_query_list;
    std::vector<std::vector<int>> all_poi_range_query_list;
    path_result.clear();
    all_poi_knn_query_list.clear();
    all_poi_range_query_list.clear();

    Kaul_Adapt_Dijk_Adapt(point_cloud, poi_list, -1, true, source_poi_index, destination_poi_index,
                          point_cloud_to_terrain_time, query_time, point_cloud_to_terrain_memory_usage,
                          memory_usage, distance_result, path_result);
    if (run_knn_query)
    {
        Kaul_Adapt_Dijk_Adapt_all_poi_knn_or_range_query(point_cloud, poi_list, -1, true, 1, k_value, range, knn_query_time, all_poi_knn_query_list);
        calculate_knn_or_range_query_error(point_cloud_exact_all_poi_knn_query_list, all_poi_knn_query_list, point_cloud_knn_query_error);
        calculate_knn_or_range_query_error(terrain_exact_all_poi_knn_query_list, all_poi_knn_query_list, terrain_knn_query_error);
    }
    if (run_range_query)
    {
        Kaul_Adapt_Dijk_Adapt_all_poi_knn_or_range_query(point_cloud, poi_list, -1, true, 2, k_value, range, range_query_time, all_poi_range_query_list);
        calculate_knn_or_range_query_error(point_cloud_exact_all_poi_range_query_list, all_poi_range_query_list, point_cloud_range_query_error);
        calculate_knn_or_range_query_error(terrain_exact_all_poi_range_query_list, all_poi_range_query_list, terrain_range_query_error);
    }

    std::cout << "Point cloud to terrain time: " << point_cloud_to_terrain_time << " ms" << std::endl;
    std::cout << "Query time: " << query_time << " ms" << std::endl;
    std::cout << "Point cloud to terrain memory usage: " << point_cloud_to_terrain_memory_usage / 1e6 << " MB" << std::endl;
    std::cout << "Memory usage: " << memory_usage / 1e6 << " MB" << std::endl;
    std::cout << "Calculated distance: " << distance_result << ", point cloud exact distance: " << point_cloud_exact_distance << ", point cloud distance error: " << distance_result / point_cloud_exact_distance - 1 << ", terrain exact distance: " << terrain_exact_distance << ", terrain distance error: " << distance_result / terrain_exact_distance - 1 << std::endl;
    if (run_knn_query)
    {
        std::cout << "Knn query time: " << knn_query_time << " ms" << std::endl;
        std::cout << "Point cloud knn error: " << point_cloud_knn_query_error << ", terrain knn error: " << terrain_knn_query_error << std::endl;
    }
    if (run_range_query)
    {
        std::cout << "Range query time: " << range_query_time << " ms" << std::endl;
        std::cout << "Point cloud range error: " << point_cloud_range_query_error << ", terrain range error: " << terrain_range_query_error << std::endl;
    }

    std::ofstream ofs("../output/output.txt", std::ios_base::app);
    ofs << "== Dijk_Adapt ==\n";
    ofs << write_file_header << "\t"
        << point_cloud_to_terrain_time << "\t"
        << 0 << "\t"
        << query_time << "\t"
        << point_cloud_to_terrain_memory_usage / 1e6 << "\t"
        << memory_usage / 1e6 << "\t"
        << 0 << "\t"
        << distance_result / point_cloud_exact_distance - 1 << "\t"
        << distance_result / terrain_exact_distance - 1 << "\t"
        << knn_query_time << "\t"
        << point_cloud_knn_query_error << "\t"
        << terrain_knn_query_error << "\t"
        << range_query_time << "\t"
        << point_cloud_range_query_error << "\t"
        << terrain_range_query_error << "\n\n";
    ofs.close();
}

void Kaul_Adapt_with_output(point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list, double epsilon,
                            int source_poi_index, int destination_poi_index, double point_cloud_exact_distance,
                            double terrain_exact_distance, bool run_knn_query, bool run_range_query,
                            int k_value, double range,
                            std::vector<std::vector<int>> &point_cloud_exact_all_poi_knn_query_list,
                            std::vector<std::vector<int>> &terrain_exact_all_poi_knn_query_list,
                            std::vector<std::vector<int>> &point_cloud_exact_all_poi_range_query_list,
                            std::vector<std::vector<int>> &terrain_exact_all_poi_range_query_list,
                            std::string write_file_header)
{
    double point_cloud_to_terrain_time = 0;
    double query_time = 0;
    double knn_query_time = 0;
    double range_query_time = 0;
    double point_cloud_to_terrain_memory_usage = 0;
    double memory_usage = 0;
    double distance_result = 0;
    double point_cloud_knn_query_error = 0;
    double terrain_knn_query_error = 0;
    double point_cloud_range_query_error = 0;
    double terrain_range_query_error = 0;
    std::vector<geodesic::SurfacePoint> path_result;
    std::vector<std::vector<int>> all_poi_knn_query_list;
    std::vector<std::vector<int>> all_poi_range_query_list;
    path_result.clear();
    all_poi_knn_query_list.clear();
    all_poi_range_query_list.clear();

    Kaul_Adapt_Dijk_Adapt(point_cloud, poi_list, epsilon, false, source_poi_index, destination_poi_index,
                          point_cloud_to_terrain_time, query_time, point_cloud_to_terrain_memory_usage,
                          memory_usage, distance_result, path_result);
    if (run_knn_query)
    {
        Kaul_Adapt_Dijk_Adapt_all_poi_knn_or_range_query(point_cloud, poi_list, epsilon, false, 1, k_value, range, knn_query_time, all_poi_knn_query_list);
        calculate_knn_or_range_query_error(point_cloud_exact_all_poi_knn_query_list, all_poi_knn_query_list, point_cloud_knn_query_error);
        calculate_knn_or_range_query_error(terrain_exact_all_poi_knn_query_list, all_poi_knn_query_list, terrain_knn_query_error);
    }
    if (run_range_query)
    {
        Kaul_Adapt_Dijk_Adapt_all_poi_knn_or_range_query(point_cloud, poi_list, epsilon, false, 2, k_value, range, range_query_time, all_poi_range_query_list);
        calculate_knn_or_range_query_error(point_cloud_exact_all_poi_range_query_list, all_poi_range_query_list, point_cloud_range_query_error);
        calculate_knn_or_range_query_error(terrain_exact_all_poi_range_query_list, all_poi_range_query_list, terrain_range_query_error);
    }

    std::cout << "Point cloud to terrain time: " << point_cloud_to_terrain_time << " ms" << std::endl;
    std::cout << "Query time: " << query_time << " ms" << std::endl;
    std::cout << "Point cloud to terrain memory usage: " << point_cloud_to_terrain_memory_usage / 1e6 << " MB" << std::endl;
    std::cout << "Memory usage: " << memory_usage / 1e6 << " MB" << std::endl;
    std::cout << "Calculated distance: " << distance_result << ", point cloud exact distance: " << point_cloud_exact_distance << ", point cloud distance error: " << distance_result / point_cloud_exact_distance - 1 << ", terrain exact distance: " << terrain_exact_distance << ", terrain distance error: " << distance_result / terrain_exact_distance - 1 << std::endl;
    if (run_knn_query)
    {
        std::cout << "Knn query time: " << knn_query_time << " ms" << std::endl;
        std::cout << "Point cloud knn error: " << point_cloud_knn_query_error << ", terrain knn error: " << terrain_knn_query_error << std::endl;
    }
    if (run_range_query)
    {
        std::cout << "Range query time: " << range_query_time << " ms" << std::endl;
        std::cout << "Point cloud range error: " << point_cloud_range_query_error << ", terrain range error: " << terrain_range_query_error << std::endl;
    }

    std::ofstream ofs("../output/output.txt", std::ios_base::app);
    ofs << "== Kaul_Adapt ==\n";
    ofs << write_file_header << "\t"
        << point_cloud_to_terrain_time << "\t"
        << 0 << "\t"
        << query_time << "\t"
        << point_cloud_to_terrain_memory_usage / 1e6 << "\t"
        << memory_usage / 1e6 << "\t"
        << 0 << "\t"
        << distance_result / point_cloud_exact_distance - 1 << "\t"
        << distance_result / terrain_exact_distance - 1 << "\t"
        << knn_query_time << "\t"
        << point_cloud_knn_query_error << "\t"
        << terrain_knn_query_error << "\t"
        << range_query_time << "\t"
        << point_cloud_range_query_error << "\t"
        << terrain_range_query_error << "\n\n";
    ofs.close();
}

void CH_Adapt_with_output(point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list,
                          int source_poi_index, int destination_poi_index, double point_cloud_exact_distance,
                          double terrain_exact_distance, bool run_knn_query, bool run_range_query,
                          int k_value, double range,
                          std::vector<std::vector<int>> &point_cloud_exact_all_poi_knn_query_list,
                          std::vector<std::vector<int>> &terrain_exact_all_poi_knn_query_list,
                          std::vector<std::vector<int>> &point_cloud_exact_all_poi_range_query_list,
                          std::vector<std::vector<int>> &terrain_exact_all_poi_range_query_list, std::string write_file_header)
{
    double point_cloud_to_terrain_time = 0;
    double query_time = 0;
    double knn_query_time = 0;
    double range_query_time = 0;
    double point_cloud_to_terrain_memory_usage = 0;
    double memory_usage = 0;
    double distance_result = 0;
    double point_cloud_knn_query_error = 0;
    double terrain_knn_query_error = 0;
    double point_cloud_range_query_error = 0;
    double terrain_range_query_error = 0;
    std::vector<geodesic::SurfacePoint> path_result;
    std::vector<std::vector<int>> all_poi_knn_query_list;
    std::vector<std::vector<int>> all_poi_range_query_list;
    path_result.clear();
    all_poi_knn_query_list.clear();
    all_poi_range_query_list.clear();

    CH_Adapt(point_cloud, poi_list, source_poi_index, destination_poi_index,
             point_cloud_to_terrain_time, query_time, point_cloud_to_terrain_memory_usage,
             memory_usage, distance_result, path_result);
    if (run_knn_query)
    {
        CH_Adapt_all_poi_knn_or_range_query(point_cloud, poi_list, 1, k_value, range, knn_query_time, all_poi_knn_query_list);
        calculate_knn_or_range_query_error(point_cloud_exact_all_poi_knn_query_list, all_poi_knn_query_list, point_cloud_knn_query_error);
        calculate_knn_or_range_query_error(terrain_exact_all_poi_knn_query_list, all_poi_knn_query_list, terrain_knn_query_error);
    }
    if (run_range_query)
    {
        CH_Adapt_all_poi_knn_or_range_query(point_cloud, poi_list, 2, k_value, range, range_query_time, all_poi_range_query_list);
        calculate_knn_or_range_query_error(point_cloud_exact_all_poi_range_query_list, all_poi_range_query_list, point_cloud_range_query_error);
        calculate_knn_or_range_query_error(terrain_exact_all_poi_range_query_list, all_poi_range_query_list, terrain_range_query_error);
    }

    std::cout << "Point cloud to terrain time: " << point_cloud_to_terrain_time << " ms" << std::endl;
    std::cout << "Query time: " << query_time << " ms" << std::endl;
    std::cout << "Point cloud to terrain memory usage: " << point_cloud_to_terrain_memory_usage / 1e6 << " MB" << std::endl;
    std::cout << "Memory usage: " << memory_usage / 1e6 << " MB" << std::endl;
    std::cout << "Calculated distance: " << distance_result << ", point cloud exact distance: " << point_cloud_exact_distance << ", point cloud distance error: " << distance_result / point_cloud_exact_distance - 1 << ", terrain exact distance: " << terrain_exact_distance << ", terrain distance error: " << distance_result / terrain_exact_distance - 1 << std::endl;
    if (run_knn_query)
    {
        std::cout << "Knn query time: " << knn_query_time << " ms" << std::endl;
        std::cout << "Point cloud knn error: " << point_cloud_knn_query_error << ", terrain knn error: " << terrain_knn_query_error << std::endl;
    }
    if (run_range_query)
    {
        std::cout << "Range query time: " << range_query_time << " ms" << std::endl;
        std::cout << "Point cloud range error: " << point_cloud_range_query_error << ", terrain range error: " << terrain_range_query_error << std::endl;
    }

    std::ofstream ofs("../output/output.txt", std::ios_base::app);
    ofs << "== CH_Adapt ==\n";
    ofs << write_file_header << "\t"
        << point_cloud_to_terrain_time << "\t"
        << 0 << "\t"
        << query_time << "\t"
        << point_cloud_to_terrain_memory_usage / 1e6 << "\t"
        << memory_usage / 1e6 << "\t"
        << 0 << "\t"
        << distance_result / point_cloud_exact_distance - 1 << "\t"
        << distance_result / terrain_exact_distance - 1 << "\t"
        << knn_query_time << "\t"
        << point_cloud_knn_query_error << "\t"
        << terrain_knn_query_error << "\t"
        << range_query_time << "\t"
        << point_cloud_range_query_error << "\t"
        << terrain_range_query_error << "\n\n";
    ofs.close();
}