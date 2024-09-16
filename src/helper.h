#include <stdio.h>
#include <stdlib.h>
#include <list>
#include <cmath>
#include <iostream>
#include <fstream>
#include <bits/stdc++.h>
#include "geodesic_algorithm_exact.h"
#include "geodesic_algorithm_subdivision.h"
#include "point_cloud.h"

void modify_path(std::vector<geodesic::SurfacePoint> &internal_path)
{
    for (int i = 1; i < internal_path.size() - 1; i++)
    {
        geodesic::SurfacePoint &prev = internal_path[i - 1];
        geodesic::SurfacePoint &curr = internal_path[i];
        geodesic::SurfacePoint &next = internal_path[i + 1];
        if (prev.type() == geodesic::VERTEX && curr.type() == geodesic::EDGE && next.type() == geodesic::VERTEX)
        {
            if (prev.distance(curr.base_element()->adjacent_vertices()[0]) + next.distance(curr.base_element()->adjacent_vertices()[0]) >
                prev.distance(curr.base_element()->adjacent_vertices()[1]) + next.distance(curr.base_element()->adjacent_vertices()[1]))
            {
                internal_path[i] = curr.base_element()->adjacent_vertices()[1];
            }
            else
            {
                internal_path[i] = curr.base_element()->adjacent_vertices()[0];
            }
        }
    }
}

void knn_or_range_query(int knn_one_range_two, int k_value, double range,
                        std::vector<std::vector<std::pair<double, int>>> &poi_to_other_poi_distance_and_index_list,
                        std::vector<std::vector<int>> &all_poi_knn_or_range_list)
{
    if (knn_one_range_two == 1)
    {
        std::vector<int> one_poi_knn_list;
        for (int i = 0; i < poi_to_other_poi_distance_and_index_list.size(); i++)
        {
            int count = 0;
            one_poi_knn_list.clear();
            for (int j = 0; j < poi_to_other_poi_distance_and_index_list.size(); j++)
            {
                if (count >= k_value)
                {
                    break;
                }
                if (poi_to_other_poi_distance_and_index_list[i][j].first != 0)
                {
                    one_poi_knn_list.push_back(poi_to_other_poi_distance_and_index_list[i][j].second);
                    count++;
                }
            }
            all_poi_knn_or_range_list.push_back(one_poi_knn_list);
        }
    }
    else if (knn_one_range_two == 2)
    {
        std::vector<int> one_poi_range_list;
        for (int i = 0; i < poi_to_other_poi_distance_and_index_list.size(); i++)
        {
            one_poi_range_list.clear();
            for (int j = 0; j < poi_to_other_poi_distance_and_index_list.size(); j++)
            {
                if (poi_to_other_poi_distance_and_index_list[i][j].first != 0 &&
                    poi_to_other_poi_distance_and_index_list[i][j].first <= range)
                {
                    one_poi_range_list.push_back(poi_to_other_poi_distance_and_index_list[i][j].second);
                }
            }
            all_poi_knn_or_range_list.push_back(one_poi_range_list);
        }
    }
}

void calculate_knn_or_range_query_error(std::vector<std::vector<int>> &exact_all_poi_knn_or_range_list,
                                        std::vector<std::vector<int>> &calculated_all_poi_knn_or_range_list,
                                        double &knn_or_range_error)
{
    int knn_or_range_error_count = 0;
    assert(exact_all_poi_knn_or_range_list.size() == calculated_all_poi_knn_or_range_list.size());
    for (int i = 0; i < exact_all_poi_knn_or_range_list.size(); i++)
    {
        assert(exact_all_poi_knn_or_range_list[i].size() == calculated_all_poi_knn_or_range_list[i].size());
        std::vector<int> a;
        std::vector<int> b;
        for (int j = 0; j < exact_all_poi_knn_or_range_list[i].size(); j++)
        {
            a.push_back(exact_all_poi_knn_or_range_list[i][j]);
            b.push_back(calculated_all_poi_knn_or_range_list[i][j]);
        }
        std::sort(a.begin(), a.end());
        std::sort(b.begin(), b.end());
        for (int j = 0; j < exact_all_poi_knn_or_range_list[i].size(); j++)
        {
            if (a[j] != b[j])
            {
                knn_or_range_error_count++;
            }
        }
    }
    knn_or_range_error = (double)knn_or_range_error_count / (double)(exact_all_poi_knn_or_range_list.size() * exact_all_poi_knn_or_range_list[0].size());
}

class GeoNode
{
public:
    int id;
    double radius;
    int index;
    GeoNode *parent;
    std::list<GeoNode *> children;
    GeoNode *covered_by;
    std::vector<std::pair<int, GeoNode *>> covers;
    GeoNode()
    {
        this->parent = NULL;
        this->children.clear();
        this->covered_by = NULL;
        this->covers.clear();
    }
    GeoNode(const int index, const double r)
    {
        this->index = index;
        this->radius = r;
        this->parent = NULL;
        this->children.clear();
        this->covered_by = NULL;
        this->covers.clear();
    }
    GeoNode(const int id, const int index, const double r)
    {
        this->id = id;
        this->index = index;
        this->radius = r;
        this->parent = NULL;
        this->children.clear();
        this->covered_by = NULL;
        this->covers.clear();
    }
    GeoNode(const GeoNode &n)
    {
        this->id = n.id;
        this->index = n.index;
        this->radius = n.radius;
        this->parent = n.parent;
        this->children.assign(n.children.begin(), n.children.end());
        this->covered_by = n.covered_by;
        this->covers.assign(n.covers.begin(), n.covers.end());
    }
    void set_id(int id)
    {
        this->id = id;
    }
    void set_parent(GeoNode *p)
    {
        this->parent = p;
        p->children.push_back(this);
    }
    void set_covered_by(GeoNode *p)
    {
        this->covered_by = p;
    }
    void set_covers(GeoNode *p)
    {
        std::pair<int, GeoNode *> q(this->index, this);
        p->covers.push_back(q);
    }
    ~GeoNode() {};
};

class GeoPair_C
{
public:
    GeoNode *node1;
    GeoNode *node2;
    double distance;
    std::vector<point_cloud_geodesic::PathPoint> path;
};

class GeoPair_T
{
public:
    GeoNode *node1;
    GeoNode *node2;
    double distance;
    std::vector<geodesic::SurfacePoint> path;
};

void hash_function_two_keys_to_one_key(int row_or_column, int i, int j, int &i_j)
{
    i_j = i * row_or_column + j;
}

void hash_function_one_key_to_two_keys(int row_or_column, int i_j, int &i, int &j)
{
    i = i_j / row_or_column;
    j = i_j % row_or_column;
}

void count_single_node(GeoNode &n, std::vector<GeoNode *> &partition_tree_to_compressed_partition_tree_to_be_removed_nodes)
{
    if (!n.children.empty())
    {
        for (std::list<GeoNode *>::iterator ite = n.children.begin(); ite != n.children.end(); ite++)
        {
            if (n.children.size() == 1 && n.parent != NULL)
            {
                partition_tree_to_compressed_partition_tree_to_be_removed_nodes.push_back(&n);
            }
            count_single_node(**ite, partition_tree_to_compressed_partition_tree_to_be_removed_nodes);
        }
    }
}

void remove_single_node(GeoNode &n, std::vector<GeoNode *> &partition_tree_to_compressed_partition_tree_to_be_removed_nodes)
{
    partition_tree_to_compressed_partition_tree_to_be_removed_nodes.clear();
    count_single_node(n, partition_tree_to_compressed_partition_tree_to_be_removed_nodes);

    for (int i = 0; i < partition_tree_to_compressed_partition_tree_to_be_removed_nodes.size(); i++)
    {
        partition_tree_to_compressed_partition_tree_to_be_removed_nodes[i]->parent->children.erase(std::remove(partition_tree_to_compressed_partition_tree_to_be_removed_nodes[i]->parent->children.begin(),
                                                                                                               partition_tree_to_compressed_partition_tree_to_be_removed_nodes[i]->parent->children.end(),
                                                                                                               partition_tree_to_compressed_partition_tree_to_be_removed_nodes[i]),
                                                                                                   partition_tree_to_compressed_partition_tree_to_be_removed_nodes[i]->parent->children.end());
        partition_tree_to_compressed_partition_tree_to_be_removed_nodes[i]->children.front()->set_parent(partition_tree_to_compressed_partition_tree_to_be_removed_nodes[i]->parent);
    }
}

void set_bottom_children_raduis(GeoNode &n, std::unordered_map<int, GeoNode *> &geo_node_in_partition_tree_unordered_map)
{
    if (n.children.empty())
    {
        n.radius = 0;
        geo_node_in_partition_tree_unordered_map[n.index] = &n;
    }
    else
    {
        for (std::list<GeoNode *>::iterator ite = n.children.begin(); ite != n.children.end(); ite++)
        {
            set_bottom_children_raduis(**ite, geo_node_in_partition_tree_unordered_map);
        }
    }
}

void partition_tree_to_compressed_partition_tree(GeoNode &n, std::vector<GeoNode *> &partition_tree_to_compressed_partition_tree_to_be_removed_nodes,
                                                 std::unordered_map<int, GeoNode *> &geo_node_in_partition_tree_unordered_map)
{
    remove_single_node(n, partition_tree_to_compressed_partition_tree_to_be_removed_nodes);
    set_bottom_children_raduis(n, geo_node_in_partition_tree_unordered_map);
}

double max(double x, double y)
{
    if (x > y)
        return x;
    return y;
}

double query_geo_C(int geo_tree_node_id, GeoNode &x, GeoNode &y,
                   int &returned_source_neighbour_index, int &returned_destination_neighbour_index,
                   std::unordered_map<int, GeoPair_C *> &geopairs,
                   std::vector<point_cloud_geodesic::PathPoint> &path_result)
{
    GeoNode *p;

    if (&x == &y)
    {
        return 0;
    }

    int x_in_geo_node_id_for_geo_pair = x.id;
    int y_in_geo_node_id_for_geo_pair = y.id;
    int x_y_in_geo_node_id_for_geo_pair;
    if (x_in_geo_node_id_for_geo_pair > y_in_geo_node_id_for_geo_pair)
    {
        int temp1 = y_in_geo_node_id_for_geo_pair;
        y_in_geo_node_id_for_geo_pair = x_in_geo_node_id_for_geo_pair;
        x_in_geo_node_id_for_geo_pair = temp1;
    }
    hash_function_two_keys_to_one_key(geo_tree_node_id, x_in_geo_node_id_for_geo_pair, y_in_geo_node_id_for_geo_pair, x_y_in_geo_node_id_for_geo_pair);

    if (geopairs.count(x_y_in_geo_node_id_for_geo_pair) != 0)
    {
        returned_source_neighbour_index = x.index;
        returned_destination_neighbour_index = y.index;
        path_result = geopairs[x_y_in_geo_node_id_for_geo_pair]->path;
        return geopairs[x_y_in_geo_node_id_for_geo_pair]->distance;
    }

    p = &x;
    while (p->parent != NULL)
    {
        p = p->parent;

        x_in_geo_node_id_for_geo_pair = (*p).id;
        y_in_geo_node_id_for_geo_pair = y.id;
        if (x_in_geo_node_id_for_geo_pair > y_in_geo_node_id_for_geo_pair)
        {
            int temp2 = y_in_geo_node_id_for_geo_pair;
            y_in_geo_node_id_for_geo_pair = x_in_geo_node_id_for_geo_pair;
            x_in_geo_node_id_for_geo_pair = temp2;
        }
        hash_function_two_keys_to_one_key(geo_tree_node_id, x_in_geo_node_id_for_geo_pair, y_in_geo_node_id_for_geo_pair, x_y_in_geo_node_id_for_geo_pair);

        if (geopairs.count(x_y_in_geo_node_id_for_geo_pair) != 0)
        {
            returned_source_neighbour_index = (*p).index;
            returned_destination_neighbour_index = y.index;
            path_result = geopairs[x_y_in_geo_node_id_for_geo_pair]->path;
            return geopairs[x_y_in_geo_node_id_for_geo_pair]->distance;
        }
    }
    p = &y;
    while (p->parent != NULL)
    {
        p = p->parent;

        x_in_geo_node_id_for_geo_pair = x.id;
        y_in_geo_node_id_for_geo_pair = (*p).id;
        if (x_in_geo_node_id_for_geo_pair > y_in_geo_node_id_for_geo_pair)
        {
            int temp3 = y_in_geo_node_id_for_geo_pair;
            y_in_geo_node_id_for_geo_pair = x_in_geo_node_id_for_geo_pair;
            x_in_geo_node_id_for_geo_pair = temp3;
        }
        hash_function_two_keys_to_one_key(geo_tree_node_id, x_in_geo_node_id_for_geo_pair, y_in_geo_node_id_for_geo_pair, x_y_in_geo_node_id_for_geo_pair);

        if (geopairs.count(x_y_in_geo_node_id_for_geo_pair) != 0)
        {
            returned_source_neighbour_index = x.index;
            returned_destination_neighbour_index = (*p).index;
            path_result = geopairs[x_y_in_geo_node_id_for_geo_pair]->path;
            return geopairs[x_y_in_geo_node_id_for_geo_pair]->distance;
        }
    }
    return query_geo_C(geo_tree_node_id, *x.parent, *y.parent, returned_source_neighbour_index, returned_destination_neighbour_index, geopairs, path_result);
}

double query_geo_T(int geo_tree_node_id, GeoNode &x, GeoNode &y,
                   int &returned_source_neighbour_index, int &returned_destination_neighbour_index,
                   std::unordered_map<int, GeoPair_T *> &geopairs,
                   std::vector<geodesic::SurfacePoint> &path_result)
{
    GeoNode *p;

    if (&x == &y)
    {
        return 0;
    }

    int x_in_geo_node_id_for_geo_pair = x.id;
    int y_in_geo_node_id_for_geo_pair = y.id;
    int x_y_in_geo_node_id_for_geo_pair;
    if (x_in_geo_node_id_for_geo_pair > y_in_geo_node_id_for_geo_pair)
    {
        int temp1 = y_in_geo_node_id_for_geo_pair;
        y_in_geo_node_id_for_geo_pair = x_in_geo_node_id_for_geo_pair;
        x_in_geo_node_id_for_geo_pair = temp1;
    }
    hash_function_two_keys_to_one_key(geo_tree_node_id, x_in_geo_node_id_for_geo_pair, y_in_geo_node_id_for_geo_pair, x_y_in_geo_node_id_for_geo_pair);

    if (geopairs.count(x_y_in_geo_node_id_for_geo_pair) != 0)
    {
        returned_source_neighbour_index = x.index;
        returned_destination_neighbour_index = y.index;
        path_result = geopairs[x_y_in_geo_node_id_for_geo_pair]->path;
        return geopairs[x_y_in_geo_node_id_for_geo_pair]->distance;
    }

    p = &x;
    while (p->parent != NULL)
    {
        p = p->parent;

        x_in_geo_node_id_for_geo_pair = (*p).id;
        y_in_geo_node_id_for_geo_pair = y.id;
        if (x_in_geo_node_id_for_geo_pair > y_in_geo_node_id_for_geo_pair)
        {
            int temp2 = y_in_geo_node_id_for_geo_pair;
            y_in_geo_node_id_for_geo_pair = x_in_geo_node_id_for_geo_pair;
            x_in_geo_node_id_for_geo_pair = temp2;
        }
        hash_function_two_keys_to_one_key(geo_tree_node_id, x_in_geo_node_id_for_geo_pair, y_in_geo_node_id_for_geo_pair, x_y_in_geo_node_id_for_geo_pair);

        if (geopairs.count(x_y_in_geo_node_id_for_geo_pair) != 0)
        {
            returned_source_neighbour_index = (*p).index;
            returned_destination_neighbour_index = y.index;
            path_result = geopairs[x_y_in_geo_node_id_for_geo_pair]->path;
            return geopairs[x_y_in_geo_node_id_for_geo_pair]->distance;
        }
    }
    p = &y;
    while (p->parent != NULL)
    {
        p = p->parent;

        x_in_geo_node_id_for_geo_pair = x.id;
        y_in_geo_node_id_for_geo_pair = (*p).id;
        if (x_in_geo_node_id_for_geo_pair > y_in_geo_node_id_for_geo_pair)
        {
            int temp3 = y_in_geo_node_id_for_geo_pair;
            y_in_geo_node_id_for_geo_pair = x_in_geo_node_id_for_geo_pair;
            x_in_geo_node_id_for_geo_pair = temp3;
        }
        hash_function_two_keys_to_one_key(geo_tree_node_id, x_in_geo_node_id_for_geo_pair, y_in_geo_node_id_for_geo_pair, x_y_in_geo_node_id_for_geo_pair);

        if (geopairs.count(x_y_in_geo_node_id_for_geo_pair) != 0)
        {
            returned_source_neighbour_index = x.index;
            returned_destination_neighbour_index = (*p).index;
            path_result = geopairs[x_y_in_geo_node_id_for_geo_pair]->path;
            return geopairs[x_y_in_geo_node_id_for_geo_pair]->distance;
        }
    }
    return query_geo_T(geo_tree_node_id, *x.parent, *y.parent, returned_source_neighbour_index, returned_destination_neighbour_index, geopairs, path_result);
}

void all_poi_knn_or_range_query_geo_C(int poi_num, int geo_tree_node_id,
                                      std::unordered_map<int, GeoNode *> &geo_node_in_partition_tree_unordered_map,
                                      std::vector<GeoNode *> &all_poi, std::unordered_map<int, GeoPair_C *> &geopairs,
                                      int knn_one_range_two, int k_value, double range,
                                      std::vector<std::vector<int>> &all_poi_knn_or_range_list)
{
    std::vector<std::vector<std::pair<double, int>>> poi_to_other_poi_distance_and_index_list;
    poi_to_other_poi_distance_and_index_list.clear();
    std::vector<std::pair<double, int>> one_poi_to_other_poi_distance_and_index_list;
    for (int i = 0; i < poi_num; i++)
    {
        one_poi_to_other_poi_distance_and_index_list.clear();
        for (int j = 0; j < poi_num; j++)
        {
            std::vector<point_cloud_geodesic::PathPoint> path_result;
            int a, b;
            double distance_result = query_geo_C(geo_tree_node_id, *geo_node_in_partition_tree_unordered_map[all_poi[i]->index], *geo_node_in_partition_tree_unordered_map[all_poi[j]->index], a, b, geopairs, path_result);

            one_poi_to_other_poi_distance_and_index_list.push_back(std::make_pair(distance_result, j));
        }
        std::sort(one_poi_to_other_poi_distance_and_index_list.begin(), one_poi_to_other_poi_distance_and_index_list.end());
        poi_to_other_poi_distance_and_index_list.push_back(one_poi_to_other_poi_distance_and_index_list);
    }
    knn_or_range_query(knn_one_range_two, k_value, range, poi_to_other_poi_distance_and_index_list, all_poi_knn_or_range_list);
}

void all_poi_knn_or_range_query_geo_T(int poi_num, int geo_tree_node_id,
                                      std::unordered_map<int, GeoNode *> &geo_node_in_partition_tree_unordered_map,
                                      std::vector<GeoNode *> &all_poi, std::unordered_map<int, GeoPair_T *> &geopairs,
                                      int knn_one_range_two, int k_value, double range,
                                      std::vector<std::vector<int>> &all_poi_knn_or_range_list)
{
    std::vector<std::vector<std::pair<double, int>>> poi_to_other_poi_distance_and_index_list;
    poi_to_other_poi_distance_and_index_list.clear();
    std::vector<std::pair<double, int>> one_poi_to_other_poi_distance_and_index_list;
    for (int i = 0; i < poi_num; i++)
    {
        one_poi_to_other_poi_distance_and_index_list.clear();
        for (int j = 0; j < poi_num; j++)
        {
            std::vector<geodesic::SurfacePoint> path_result;
            int a, b;
            double distance_result = query_geo_T(geo_tree_node_id, *geo_node_in_partition_tree_unordered_map[all_poi[i]->index], *geo_node_in_partition_tree_unordered_map[all_poi[j]->index], a, b, geopairs, path_result);

            one_poi_to_other_poi_distance_and_index_list.push_back(std::make_pair(distance_result, j));
        }
        std::sort(one_poi_to_other_poi_distance_and_index_list.begin(), one_poi_to_other_poi_distance_and_index_list.end());
        poi_to_other_poi_distance_and_index_list.push_back(one_poi_to_other_poi_distance_and_index_list);
    }
    knn_or_range_query(knn_one_range_two, k_value, range, poi_to_other_poi_distance_and_index_list, all_poi_knn_or_range_list);
}

void sort_min_to_max_and_get_original_index(std::vector<double> &vec, std::vector<std::pair<double, int>> &vp)
{
    for (int i = 0; i < vec.size(); i++)
    {
        vp.push_back(std::make_pair(vec[i], i));
    }
    std::sort(vp.begin(), vp.end());
}

double euclidean_distance(double x_1, double y_1, double x_2, double y_2)
{
    return sqrt(pow(x_1 - x_2, 2) + pow(y_1 - y_2, 2));
}

double e_to_subdivision_level(double e)
{
    assert(e > 0 && e <= 1);
    double subdivision_level = 0;
    if (e > 0 && e <= 0.05)
    {
        subdivision_level = floor(7 / (10 * e)) - 8;
    }
    else if (e > 0.05 && e <= 0.1)
    {
        subdivision_level = floor(7 / (10 * e)) - 2;
    }
    else if (e > 0.1 && e <= 0.25)
    {
        subdivision_level = floor(1 / e) - 1;
    }
    else if (e > 0.25 && e < 1)
    {
        subdivision_level = floor(1 / e);
    }
    if (subdivision_level < 5)
    {
        subdivision_level++;
    }
    std::cout << "subdivision_level: " << subdivision_level << std::endl;
    return subdivision_level;
}

void pre_compute_Point(int poi_num, point_cloud_geodesic::PointCloud *point_cloud, std::vector<int> &poi_list,
                       std::unordered_map<int, double> &pairwise_distance_poi_to_poi_map,
                       std::unordered_map<int, std::vector<point_cloud_geodesic::PathPoint>> &pairwise_path_poi_to_poi_map,
                       double &memory_usage)
{
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
}

void pre_compute_FaceExact(int poi_num, geodesic::Mesh *mesh, std::vector<int> &poi_list,
                           std::unordered_map<int, double> &pairwise_distance_poi_to_poi_map,
                           std::unordered_map<int, std::vector<geodesic::SurfacePoint>> &pairwise_path_poi_to_poi_map,
                           double &memory_usage)
{
    geodesic::GeodesicAlgorithmExact algorithm(mesh);
    double const distance_limit = geodesic::GEODESIC_INF;
    pairwise_distance_poi_to_poi_map.clear();
    pairwise_path_poi_to_poi_map.clear();
    int pairwise_path_poi_to_poi_size = 0;
    std::vector<geodesic::SurfacePoint> one_source_poi_list;
    std::vector<geodesic::SurfacePoint> destinations_poi_list;

    for (int i = 0; i < poi_num; i++)
    {
        one_source_poi_list.clear();
        destinations_poi_list.clear();
        one_source_poi_list.push_back(geodesic::SurfacePoint(&mesh->vertices()[poi_list[i]]));
        for (int j = i; j < poi_num; j++)
        {
            destinations_poi_list.push_back(geodesic::SurfacePoint(&mesh->vertices()[poi_list[j]]));
        }
        algorithm.propagate(one_source_poi_list, distance_limit);
        for (int j = i; j < poi_num; j++)
        {
            std::vector<geodesic::SurfacePoint> path;
            geodesic::SurfacePoint one_destination(&mesh->vertices()[poi_list[j]]);
            algorithm.trace_back(one_destination, path);
            int i_j;
            hash_function_two_keys_to_one_key(poi_num, i, j, i_j);
            pairwise_distance_poi_to_poi_map[i_j] = length(path);
            pairwise_path_poi_to_poi_map[i_j] = path;
            pairwise_path_poi_to_poi_size += path.size();
        }
    }
    memory_usage += algorithm.get_memory() + 0.5 * poi_num * (poi_num - 1) * sizeof(double) + pairwise_path_poi_to_poi_size * sizeof(geodesic::SurfacePoint);
}

void build_level_C(int &geo_tree_node_id, point_cloud_geodesic::PointCloud *point_cloud, int depth,
                   stx::btree<int, GeoNode *> &pois_B_tree, stx::btree<int, GeoNode *> &pois_as_center_each_parent_layer,
                   std::unordered_map<int, double> &pre_pairwise_distance_poi_to_poi_map,
                   std::unordered_map<int, std::vector<point_cloud_geodesic::PathPoint>> &pre_pairwise_path_poi_to_poi_map,
                   std::unordered_map<int, int> &poi_unordered_map)
{
    std::vector<std::pair<int, GeoNode *>> pois_as_center_each_current_layer;
    pois_as_center_each_current_layer.clear();

    for (stx::btree<int, GeoNode *>::iterator ite = pois_as_center_each_parent_layer.begin(); ite != pois_as_center_each_parent_layer.end(); ite++)
    {
        std::vector<std::pair<int, GeoNode *>> current_parent_covers_but_remained_pois = (*ite).second->covers;
        GeoNode *n = new GeoNode(geo_tree_node_id, (*ite).second->index, ((*ite).second)->radius / 2.0);
        geo_tree_node_id++;
        n->set_parent((*ite).second);
        std::pair<int, GeoNode *> m(n->index, n);
        pois_as_center_each_current_layer.push_back(m);
        double const distance_limit = n->radius;
        auto bite = current_parent_covers_but_remained_pois.begin();

        while (bite != current_parent_covers_but_remained_pois.end())
        {
            int x_in_poi_list = poi_unordered_map[n->index];
            int y_in_poi_list = poi_unordered_map[(*bite).first];
            int x_y_in_poi_list;
            for (int i = 0; i < poi_unordered_map.size() * 100; i++)
            {
                if (x_in_poi_list <= y_in_poi_list)
                {
                    hash_function_two_keys_to_one_key(poi_unordered_map.size(), x_in_poi_list, y_in_poi_list, x_y_in_poi_list);
                }
                else
                {
                    hash_function_two_keys_to_one_key(poi_unordered_map.size(), y_in_poi_list, x_in_poi_list, x_y_in_poi_list);
                }
            }

            if (pre_pairwise_distance_poi_to_poi_map[x_y_in_poi_list] <= distance_limit)
            {
                (*bite).second->set_covered_by(n);
                (*bite).second->set_covers(n);
                current_parent_covers_but_remained_pois.erase(bite);
            }
            else
            {
                bite++;
            }
        }

        while (!current_parent_covers_but_remained_pois.empty())
        {
            GeoNode *a = new GeoNode(geo_tree_node_id, (*current_parent_covers_but_remained_pois.begin()).second->index, (*current_parent_covers_but_remained_pois.begin()).second->covered_by->radius / 2.0);
            geo_tree_node_id++;
            a->set_parent((*ite).second);
            current_parent_covers_but_remained_pois.erase(current_parent_covers_but_remained_pois.begin());
            std::pair<int, GeoNode *> b(a->index, a);
            pois_as_center_each_current_layer.push_back(b);
            double const distance_limit = b.second->radius;
            auto bite = current_parent_covers_but_remained_pois.begin();

            while (bite != current_parent_covers_but_remained_pois.end())
            {
                int x_in_poi_list = poi_unordered_map[a->index];
                int y_in_poi_list = poi_unordered_map[(*bite).first];
                int x_y_in_poi_list;
                for (int i = 0; i < poi_unordered_map.size() * 100; i++)
                {
                    if (x_in_poi_list <= y_in_poi_list)
                    {
                        hash_function_two_keys_to_one_key(poi_unordered_map.size(), x_in_poi_list, y_in_poi_list, x_y_in_poi_list);
                    }
                    else
                    {
                        hash_function_two_keys_to_one_key(poi_unordered_map.size(), y_in_poi_list, x_in_poi_list, x_y_in_poi_list);
                    }
                }

                if (pre_pairwise_distance_poi_to_poi_map[x_y_in_poi_list] <= distance_limit)
                {
                    (*bite).second->set_covered_by(a);
                    (*bite).second->set_covers(a);
                    current_parent_covers_but_remained_pois.erase(bite);
                }
                else
                {
                    bite++;
                }
            }
        }
    }
    pois_as_center_each_parent_layer.clear();
    pois_as_center_each_parent_layer = stx::btree<int, GeoNode *>(pois_as_center_each_current_layer.begin(), pois_as_center_each_current_layer.end());
}

void build_geo_tree_C(int &geo_tree_node_id, point_cloud_geodesic::PointCloud *point_cloud, GeoNode &root_geo, int poi_num,
                      stx::btree<int, GeoNode *> &pois_B_tree, stx::btree<int, GeoNode *> &pois_as_center_each_parent_layer,
                      std::unordered_map<int, double> &pre_pairwise_distance_poi_to_poi_map,
                      std::unordered_map<int, std::vector<point_cloud_geodesic::PathPoint>> &pre_pairwise_path_poi_to_poi_map,
                      std::unordered_map<int, int> &poi_unordered_map)
{
    int depth = 0;
    pois_as_center_each_parent_layer.clear();
    pois_as_center_each_parent_layer.insert(root_geo.index, &root_geo);
    stx::btree<int, GeoNode *> remained_pois = stx::btree<int, GeoNode *>(pois_B_tree);
    remained_pois.erase(root_geo.index);
    double const distance_limit = root_geo.radius;

    for (stx::btree<int, GeoNode *>::iterator bite = remained_pois.begin(); bite != remained_pois.end(); bite++)
    {
        int x_in_poi_list = poi_unordered_map[root_geo.index];
        int y_in_poi_list = poi_unordered_map[(*bite).first];
        int x_y_in_poi_list;
        for (int i = 0; i < poi_unordered_map.size() * 100; i++)
        {
            if (x_in_poi_list <= y_in_poi_list)
            {
                hash_function_two_keys_to_one_key(poi_num, x_in_poi_list, y_in_poi_list, x_y_in_poi_list);
            }
            else
            {
                hash_function_two_keys_to_one_key(poi_num, y_in_poi_list, x_in_poi_list, x_y_in_poi_list);
            }
        }

        if (pre_pairwise_distance_poi_to_poi_map[x_y_in_poi_list] <= distance_limit)
        {
            (*bite).second->set_covered_by(&root_geo);
            (*bite).second->set_covers(&root_geo);
        }
    }

    while (pois_as_center_each_parent_layer.size() != poi_num)
    {
        build_level_C(geo_tree_node_id, point_cloud, depth, pois_B_tree, pois_as_center_each_parent_layer, pre_pairwise_distance_poi_to_poi_map, pre_pairwise_path_poi_to_poi_map, poi_unordered_map);
        depth++;
    }
}

void build_level_T(int &geo_tree_node_id, geodesic::Mesh *mesh, int depth,
                   stx::btree<int, GeoNode *> &pois_B_tree, stx::btree<int, GeoNode *> &pois_as_center_each_parent_layer,
                   std::unordered_map<int, double> &pre_pairwise_distance_poi_to_poi_map,
                   std::unordered_map<int, std::vector<geodesic::SurfacePoint>> &pre_pairwise_path_poi_to_poi_map,
                   std::unordered_map<int, int> &poi_unordered_map)
{
    std::vector<std::pair<int, GeoNode *>> pois_as_center_each_current_layer;
    pois_as_center_each_current_layer.clear();

    for (stx::btree<int, GeoNode *>::iterator ite = pois_as_center_each_parent_layer.begin(); ite != pois_as_center_each_parent_layer.end(); ite++)
    {
        std::vector<std::pair<int, GeoNode *>> current_parent_covers_but_remained_pois = (*ite).second->covers;
        GeoNode *n = new GeoNode(geo_tree_node_id, (*ite).second->index, ((*ite).second)->radius / 2.0);
        geo_tree_node_id++;
        n->set_parent((*ite).second);
        std::pair<int, GeoNode *> m(n->index, n);
        pois_as_center_each_current_layer.push_back(m);
        double const distance_limit = n->radius;
        auto bite = current_parent_covers_but_remained_pois.begin();

        while (bite != current_parent_covers_but_remained_pois.end())
        {
            int x_in_poi_list = poi_unordered_map[n->index];
            int y_in_poi_list = poi_unordered_map[(*bite).first];
            int x_y_in_poi_list;
            for (int i = 0; i < poi_unordered_map.size() * 100; i++)
            {
                if (x_in_poi_list <= y_in_poi_list)
                {
                    hash_function_two_keys_to_one_key(poi_unordered_map.size(), x_in_poi_list, y_in_poi_list, x_y_in_poi_list);
                }
                else
                {
                    hash_function_two_keys_to_one_key(poi_unordered_map.size(), y_in_poi_list, x_in_poi_list, x_y_in_poi_list);
                }
            }

            if (pre_pairwise_distance_poi_to_poi_map[x_y_in_poi_list] <= distance_limit)
            {
                (*bite).second->set_covered_by(n);
                (*bite).second->set_covers(n);
                current_parent_covers_but_remained_pois.erase(bite);
            }
            else
            {
                bite++;
            }
        }

        while (!current_parent_covers_but_remained_pois.empty())
        {
            GeoNode *a = new GeoNode(geo_tree_node_id, (*current_parent_covers_but_remained_pois.begin()).second->index, (*current_parent_covers_but_remained_pois.begin()).second->covered_by->radius / 2.0);
            geo_tree_node_id++;
            a->set_parent((*ite).second);
            current_parent_covers_but_remained_pois.erase(current_parent_covers_but_remained_pois.begin());
            std::pair<int, GeoNode *> b(a->index, a);
            pois_as_center_each_current_layer.push_back(b);
            double const distance_limit = b.second->radius;
            auto bite = current_parent_covers_but_remained_pois.begin();

            while (bite != current_parent_covers_but_remained_pois.end())
            {
                int x_in_poi_list = poi_unordered_map[a->index];
                int y_in_poi_list = poi_unordered_map[(*bite).first];
                int x_y_in_poi_list;
                for (int i = 0; i < poi_unordered_map.size() * 100; i++)
                {
                    if (x_in_poi_list <= y_in_poi_list)
                    {
                        hash_function_two_keys_to_one_key(poi_unordered_map.size(), x_in_poi_list, y_in_poi_list, x_y_in_poi_list);
                    }
                    else
                    {
                        hash_function_two_keys_to_one_key(poi_unordered_map.size(), y_in_poi_list, x_in_poi_list, x_y_in_poi_list);
                    }
                }

                if (pre_pairwise_distance_poi_to_poi_map[x_y_in_poi_list] <= distance_limit)
                {
                    (*bite).second->set_covered_by(a);
                    (*bite).second->set_covers(a);
                    current_parent_covers_but_remained_pois.erase(bite);
                }
                else
                {
                    bite++;
                }
            }
        }
    }
    pois_as_center_each_parent_layer.clear();
    pois_as_center_each_parent_layer = stx::btree<int, GeoNode *>(pois_as_center_each_current_layer.begin(), pois_as_center_each_current_layer.end());
}

void build_geo_tree_T(int &geo_tree_node_id, geodesic::Mesh *mesh, GeoNode &root_geo, int poi_num,
                      stx::btree<int, GeoNode *> &pois_B_tree, stx::btree<int, GeoNode *> &pois_as_center_each_parent_layer,
                      std::unordered_map<int, double> &pre_pairwise_distance_poi_to_poi_map,
                      std::unordered_map<int, std::vector<geodesic::SurfacePoint>> &pre_pairwise_path_poi_to_poi_map,
                      std::unordered_map<int, int> &poi_unordered_map)
{
    int depth = 0;
    pois_as_center_each_parent_layer.clear();
    pois_as_center_each_parent_layer.insert(root_geo.index, &root_geo);
    stx::btree<int, GeoNode *> remained_pois = stx::btree<int, GeoNode *>(pois_B_tree);
    remained_pois.erase(root_geo.index);
    double const distance_limit = root_geo.radius;

    for (stx::btree<int, GeoNode *>::iterator bite = remained_pois.begin(); bite != remained_pois.end(); bite++)
    {
        int x_in_poi_list = poi_unordered_map[root_geo.index];
        int y_in_poi_list = poi_unordered_map[(*bite).first];
        int x_y_in_poi_list;
        for (int i = 0; i < poi_unordered_map.size() * 100; i++)
        {
            if (x_in_poi_list <= y_in_poi_list)
            {
                hash_function_two_keys_to_one_key(poi_num, x_in_poi_list, y_in_poi_list, x_y_in_poi_list);
            }
            else
            {
                hash_function_two_keys_to_one_key(poi_num, y_in_poi_list, x_in_poi_list, x_y_in_poi_list);
            }
        }

        if (pre_pairwise_distance_poi_to_poi_map[x_y_in_poi_list] <= distance_limit)
        {
            (*bite).second->set_covered_by(&root_geo);
            (*bite).second->set_covers(&root_geo);
        }
    }

    while (pois_as_center_each_parent_layer.size() != poi_num)
    {
        build_level_T(geo_tree_node_id, mesh, depth, pois_B_tree, pois_as_center_each_parent_layer, pre_pairwise_distance_poi_to_poi_map, pre_pairwise_path_poi_to_poi_map, poi_unordered_map);
        depth++;
    }
}

void generate_geo_pair_C(int geo_tree_node_id, int &WSPD_oracle_edge_num,
                         point_cloud_geodesic::PointCloud *point_cloud, GeoNode &x, GeoNode &y,
                         double WSPD_oracle_e, std::unordered_map<int, GeoPair_C *> &geopairs,
                         std::unordered_map<int, int> &poi_unordered_map,
                         std::unordered_map<int, int> &geo_pair_unordered_map,
                         std::unordered_map<int, double> &pre_pairwise_distance_poi_to_poi_map,
                         std::unordered_map<int, std::vector<point_cloud_geodesic::PathPoint>> &pre_pairwise_path_poi_to_poi_map,
                         int &pairwise_path_poi_to_poi_size)
{
    int x_in_geo_node_id = x.id;
    int y_in_geo_node_id = y.id;
    int x_y_in_geo_node_id;
    if (x_in_geo_node_id > y_in_geo_node_id)
    {
        int temp1 = y_in_geo_node_id;
        y_in_geo_node_id = x_in_geo_node_id;
        x_in_geo_node_id = temp1;
    }
    for (int i = 0; i < poi_unordered_map.size() * 100; i++)
    {
        hash_function_two_keys_to_one_key(geo_tree_node_id, x_in_geo_node_id, y_in_geo_node_id, x_y_in_geo_node_id);
    }

    if (geo_pair_unordered_map.count(x_y_in_geo_node_id) == 0)
    {
        geo_pair_unordered_map[x_y_in_geo_node_id] = 1;

        assert(poi_unordered_map.count(x.index) != 0 && poi_unordered_map.count(y.index) != 0);
        int x_in_poi_list = poi_unordered_map[x.index];
        int y_in_poi_list = poi_unordered_map[y.index];
        int x_y_in_poi_list;
        for (int i = 0; i < poi_unordered_map.size() * 100; i++)
        {
            if (x_in_poi_list <= y_in_poi_list)
            {
                hash_function_two_keys_to_one_key(poi_unordered_map.size(), x_in_poi_list, y_in_poi_list, x_y_in_poi_list);
            }
            else
            {
                hash_function_two_keys_to_one_key(poi_unordered_map.size(), y_in_poi_list, x_in_poi_list, x_y_in_poi_list);
            }
        }

        double distancexy = pre_pairwise_distance_poi_to_poi_map[x_y_in_poi_list];
        std::vector<point_cloud_geodesic::PathPoint> pathxy = pre_pairwise_path_poi_to_poi_map[x_y_in_poi_list];

        if (x.radius == 0 && y.radius == 0)
        {
            if (&x == &y)
            {
                return;
            }
            if (x.index == y.index)
            {
                return;
            }
        }
        if (distancexy >= (2.0 / WSPD_oracle_e + 2.0) * max(x.radius, y.radius))
        {
            WSPD_oracle_edge_num++;
            GeoPair_C *nodepair = new GeoPair_C();
            nodepair->node1 = &x;
            nodepair->node2 = &y;
            nodepair->distance = distancexy;
            nodepair->path = pathxy;
            pairwise_path_poi_to_poi_size += pathxy.size();

            int x_in_geo_node_id_for_geo_pair = x.id;
            int y_in_geo_node_id_for_geo_pair = y.id;
            int x_y_in_geo_node_id_for_geo_pair;
            if (x_in_geo_node_id_for_geo_pair > y_in_geo_node_id_for_geo_pair)
            {
                int temp3 = y_in_geo_node_id_for_geo_pair;
                y_in_geo_node_id_for_geo_pair = x_in_geo_node_id_for_geo_pair;
                x_in_geo_node_id_for_geo_pair = temp3;
            }
            for (int i = 0; i < poi_unordered_map.size() * 100; i++)
            {
                hash_function_two_keys_to_one_key(geo_tree_node_id, x_in_geo_node_id_for_geo_pair, y_in_geo_node_id_for_geo_pair, x_y_in_geo_node_id_for_geo_pair);
            }
            geopairs[x_y_in_geo_node_id_for_geo_pair] = nodepair;
        }
        else
        {
            if (x.radius > y.radius)
            {
                for (std::list<GeoNode *>::iterator ite = x.children.begin(); ite != x.children.end(); ite++)
                {
                    generate_geo_pair_C(geo_tree_node_id, WSPD_oracle_edge_num, point_cloud, (**ite), y, WSPD_oracle_e, geopairs, poi_unordered_map, geo_pair_unordered_map, pre_pairwise_distance_poi_to_poi_map, pre_pairwise_path_poi_to_poi_map, pairwise_path_poi_to_poi_size);
                }
            }
            else
            {
                for (std::list<GeoNode *>::iterator jte = y.children.begin(); jte != y.children.end(); jte++)
                {
                    generate_geo_pair_C(geo_tree_node_id, WSPD_oracle_edge_num, point_cloud, x, (**jte), WSPD_oracle_e, geopairs, poi_unordered_map, geo_pair_unordered_map, pre_pairwise_distance_poi_to_poi_map, pre_pairwise_path_poi_to_poi_map, pairwise_path_poi_to_poi_size);
                }
            }
        }
    }
}

void generate_geo_pair_T(int geo_tree_node_id, int &WSPD_oracle_edge_num,
                         geodesic::Mesh *mesh, GeoNode &x, GeoNode &y,
                         double WSPD_oracle_e,
                         std::unordered_map<int, GeoPair_T *> &geopairs,
                         std::unordered_map<int, int> &poi_unordered_map,
                         std::unordered_map<int, int> &geo_pair_unordered_map,
                         std::unordered_map<int, double> &pre_pairwise_distance_poi_to_poi_map,
                         std::unordered_map<int, std::vector<geodesic::SurfacePoint>> &pre_pairwise_path_poi_to_poi_map,
                         int &pairwise_path_poi_to_poi_size)
{
    int x_in_geo_node_id = x.id;
    int y_in_geo_node_id = y.id;
    int x_y_in_geo_node_id;
    if (x_in_geo_node_id > y_in_geo_node_id)
    {
        int temp1 = y_in_geo_node_id;
        y_in_geo_node_id = x_in_geo_node_id;
        x_in_geo_node_id = temp1;
    }
    for (int i = 0; i < poi_unordered_map.size() * 100; i++)
    {
        hash_function_two_keys_to_one_key(geo_tree_node_id, x_in_geo_node_id, y_in_geo_node_id, x_y_in_geo_node_id);
    }

    if (geo_pair_unordered_map.count(x_y_in_geo_node_id) == 0)
    {
        geo_pair_unordered_map[x_y_in_geo_node_id] = 1;

        assert(poi_unordered_map.count(x.index) != 0 && poi_unordered_map.count(y.index) != 0);
        int x_in_poi_list = poi_unordered_map[x.index];
        int y_in_poi_list = poi_unordered_map[y.index];
        int x_y_in_poi_list;
        for (int i = 0; i < poi_unordered_map.size() * 100; i++)
        {
            if (x_in_poi_list <= y_in_poi_list)
            {
                hash_function_two_keys_to_one_key(poi_unordered_map.size(), x_in_poi_list, y_in_poi_list, x_y_in_poi_list);
            }
            else
            {
                hash_function_two_keys_to_one_key(poi_unordered_map.size(), y_in_poi_list, x_in_poi_list, x_y_in_poi_list);
            }
        }

        double distancexy = pre_pairwise_distance_poi_to_poi_map[x_y_in_poi_list];
        std::vector<geodesic::SurfacePoint> pathxy = pre_pairwise_path_poi_to_poi_map[x_y_in_poi_list];

        if (x.radius == 0 && y.radius == 0)
        {
            if (&x == &y)
            {
                return;
            }
            if (x.index == y.index)
            {
                return;
            }
        }
        if (distancexy >= (2.0 / WSPD_oracle_e + 2.0) * max(x.radius, y.radius))
        {
            WSPD_oracle_edge_num++;
            GeoPair_T *nodepair = new GeoPair_T();
            nodepair->node1 = &x;
            nodepair->node2 = &y;
            nodepair->distance = distancexy;
            nodepair->path = pathxy;
            pairwise_path_poi_to_poi_size += pathxy.size();

            int x_in_geo_node_id_for_geo_pair = x.id;
            int y_in_geo_node_id_for_geo_pair = y.id;
            int x_y_in_geo_node_id_for_geo_pair;
            if (x_in_geo_node_id_for_geo_pair > y_in_geo_node_id_for_geo_pair)
            {
                int temp3 = y_in_geo_node_id_for_geo_pair;
                y_in_geo_node_id_for_geo_pair = x_in_geo_node_id_for_geo_pair;
                x_in_geo_node_id_for_geo_pair = temp3;
            }
            for (int i = 0; i < poi_unordered_map.size() * 100; i++)
            {
                hash_function_two_keys_to_one_key(geo_tree_node_id, x_in_geo_node_id_for_geo_pair, y_in_geo_node_id_for_geo_pair, x_y_in_geo_node_id_for_geo_pair);
            }
            geopairs[x_y_in_geo_node_id_for_geo_pair] = nodepair;
        }
        else
        {
            if (x.radius > y.radius)
            {
                for (std::list<GeoNode *>::iterator ite = x.children.begin(); ite != x.children.end(); ite++)
                {
                    generate_geo_pair_T(geo_tree_node_id, WSPD_oracle_edge_num, mesh, (**ite), y, WSPD_oracle_e, geopairs, poi_unordered_map, geo_pair_unordered_map, pre_pairwise_distance_poi_to_poi_map, pre_pairwise_path_poi_to_poi_map, pairwise_path_poi_to_poi_size);
                }
            }
            else
            {
                for (std::list<GeoNode *>::iterator jte = y.children.begin(); jte != y.children.end(); jte++)
                {
                    generate_geo_pair_T(geo_tree_node_id, WSPD_oracle_edge_num, mesh, x, (**jte), WSPD_oracle_e, geopairs, poi_unordered_map, geo_pair_unordered_map, pre_pairwise_distance_poi_to_poi_map, pre_pairwise_path_poi_to_poi_map, pairwise_path_poi_to_poi_size);
                }
            }
        }
    }
}

int doubleCmp(const double &x)
{
    if (fabs(x) < 1e-5)
        return 0;
    return x < 0 ? -1 : 1;
}

double cal_factor(double e, int type)
{
    if (type == 1)
    {
        return e < 0.1 ? 2 : (e >= 0.75 ? 0.5 : 1);
    }
    else if (type == 2)
    {
        return e >= 0.75 ? 0.5 : 1;
    }
    else if (type == 3)
    {
        return e < 0.1 ? 2 : 1;
    }
    return 1;
}

int cal_iteration(int a, int b)
{
    return a / b / (b > 300 ? 10 : 1);
}

int cross_product(double p1_x, double p1_y, double p2_x, double p2_y)
{
    return doubleCmp(p1_x * p2_y - p2_x * p1_y);
}

bool segment_intersection(double a_x, double a_y, double b_x, double b_y,
                          double c_x, double c_y, double d_x, double d_y)
{
    if (doubleCmp(std::max(a_x, b_x) - std::min(c_x, d_x)) < 0 || doubleCmp(std::max(a_y, b_y) - std::min(c_y, d_y)) < 0 ||
        doubleCmp(std::max(c_x, d_x) - std::min(a_x, b_x)) < 0 || doubleCmp(std::max(c_y, d_y) - std::min(a_y, b_y)) < 0)
        return false;
    float dir1 = (c_x - a_x) * (b_y - a_y) - (b_x - a_x) * (c_y - a_y);
    float dir2 = (d_x - a_x) * (b_y - a_y) - (b_x - a_x) * (d_y - a_y);
    float dir3 = (a_x - c_x) * (d_y - c_y) - (d_x - c_x) * (a_y - c_y);
    float dir4 = (b_x - c_x) * (d_y - c_y) - (d_x - c_x) * (b_y - c_y);
    return doubleCmp(dir1 * dir2) <= 0 && doubleCmp(dir3 * dir4) <= 0;
}

//  triangle:ABC, segment:PQ
bool triangle_segment_intersection(double a_x, double a_y, double b_x, double b_y,
                                   double c_x, double c_y, double p_x, double p_y,
                                   double q_x, double q_y)
{
    int dir1 = cross_product(b_x - a_x, b_y - a_y, p_x - a_x, p_y - a_y);
    int dir2 = cross_product(c_x - b_x, c_y - b_y, p_x - b_x, p_y - b_y);
    int dir3 = cross_product(a_x - c_x, a_y - c_y, p_x - c_x, p_y - c_y);
    bool flag_p = false, flag_q = false;
    if (dir1 == dir2 && dir2 == dir3)
    {
        flag_p = true;
    }
    dir1 = cross_product(b_x - a_x, b_y - a_y, q_x - a_x, q_y - a_y);
    dir2 = cross_product(c_x - b_x, c_y - b_y, q_x - b_x, q_y - b_y);
    dir3 = cross_product(a_x - c_x, a_y - c_y, q_x - c_x, q_y - c_y);
    if (dir1 == dir2 && dir2 == dir3)
    {
        flag_q = true;
    }
    if (flag_p && flag_q)
    {
        return true;
    }
    return segment_intersection(a_x, a_y, b_x, b_y, p_x, p_y, q_x, q_y) ||
           segment_intersection(b_x, b_y, c_x, c_y, p_x, p_y, q_x, q_y) ||
           segment_intersection(c_x, c_y, a_x, a_y, p_x, p_y, q_x, q_y);
}

void divide_mesh_into_box(geodesic::Mesh *mesh, int sqrt_num_of_box,
                          std::unordered_map<int, int> &highway_node_id_map,
                          std::unordered_map<int, std::unordered_map<int, int>> &highway_node_id_with_box_id_map)
{
    double box_width = mesh->m_width / sqrt_num_of_box;
    double box_height = mesh->m_height / sqrt_num_of_box;
    int box_id = 0;

    for (int i = 0; i < sqrt_num_of_box; i++)
    {
        for (int j = 0; j < sqrt_num_of_box; j++)
        {
            double x_min = mesh->m_xmin + i * box_width;
            double x_max = mesh->m_xmin + (i + 1) * box_width;
            double y_min = mesh->m_ymin + j * box_height;
            double y_max = mesh->m_ymin + (j + 1) * box_height;

            std::unordered_map<int, int> one_highway_node_id;
            one_highway_node_id.clear();

            for (int k = 0; k < mesh->faces().size(); k++)
            {
                double a_x = mesh->faces()[k].adjacent_vertices()[0]->getx();
                double a_y = mesh->faces()[k].adjacent_vertices()[0]->gety();
                double b_x = mesh->faces()[k].adjacent_vertices()[1]->getx();
                double b_y = mesh->faces()[k].adjacent_vertices()[1]->gety();
                double c_x = mesh->faces()[k].adjacent_vertices()[2]->getx();
                double c_y = mesh->faces()[k].adjacent_vertices()[2]->gety();

                if (triangle_segment_intersection(a_x, a_y, b_x, b_y, c_x, c_y, x_min, y_min, x_max, y_min) ||
                    triangle_segment_intersection(a_x, a_y, b_x, b_y, c_x, c_y, x_min, y_min, x_min, y_max) ||
                    triangle_segment_intersection(a_x, a_y, b_x, b_y, c_x, c_y, x_min, y_max, x_max, y_max) ||
                    triangle_segment_intersection(a_x, a_y, b_x, b_y, c_x, c_y, x_max, y_min, x_max, y_max))
                {
                    int v1_id = mesh->faces()[k].adjacent_vertices()[0]->id();
                    int v2_id = mesh->faces()[k].adjacent_vertices()[1]->id();
                    int v3_id = mesh->faces()[k].adjacent_vertices()[2]->id();
                    one_highway_node_id[v1_id] = v1_id;
                    one_highway_node_id[v2_id] = v2_id;
                    one_highway_node_id[v3_id] = v3_id;
                    highway_node_id_map[v1_id] = v1_id;
                    highway_node_id_map[v2_id] = v2_id;
                    highway_node_id_map[v3_id] = v3_id;
                }
            }
            highway_node_id_with_box_id_map[box_id] = one_highway_node_id;
            box_id++;
        }
    }
}

void pre_compute_EAR_Oracle_highway_node_C(point_cloud_geodesic::PointCloud *point_cloud,
                                           std::vector<int> &highway_node_list,
                                           std::unordered_map<int, double> &distance_highway_node_to_highway_node_map,
                                           std::unordered_map<int, std::vector<point_cloud_geodesic::PathPoint>> &path_highway_node_to_highway_node_map,
                                           double &memory_usage)
{
    point_cloud_geodesic::PointCloudGeodesicAlgorithmDijkstra algorithm(point_cloud);
    double const distance_limit = point_cloud_geodesic::INFIN;
    distance_highway_node_to_highway_node_map.clear();
    path_highway_node_to_highway_node_map.clear();
    int path_highway_node_to_highway_node_size = 0;
    std::vector<point_cloud_geodesic::PathPoint> one_source_highway_node_list;
    std::vector<point_cloud_geodesic::PathPoint> destinations_highway_node_list;

    for (int i = 0; i < highway_node_list.size(); i++)
    {
        one_source_highway_node_list.clear();
        destinations_highway_node_list.clear();
        one_source_highway_node_list.push_back(point_cloud_geodesic::PathPoint(&point_cloud->pc_points()[highway_node_list[i]]));
        for (int j = i; j < highway_node_list.size(); j++)
        {
            destinations_highway_node_list.push_back(point_cloud_geodesic::PathPoint(&point_cloud->pc_points()[highway_node_list[j]]));
        }
        algorithm.propagate(one_source_highway_node_list, distance_limit);
        for (int j = i; j < highway_node_list.size(); j++)
        {
            std::vector<point_cloud_geodesic::PathPoint> path;
            point_cloud_geodesic::PathPoint one_destination(&point_cloud->pc_points()[highway_node_list[j]]);
            algorithm.trace_back(one_destination, path);
            int i_j;
            hash_function_two_keys_to_one_key(highway_node_list.size(), i, j, i_j);
            distance_highway_node_to_highway_node_map[i_j] = length(path);
            path_highway_node_to_highway_node_map[i_j] = path;
            path_highway_node_to_highway_node_size += path.size();
        }
    }
    memory_usage += algorithm.get_memory() + 0.5 * highway_node_list.size() * (highway_node_list.size() - 1) * sizeof(double) + path_highway_node_to_highway_node_size * sizeof(point_cloud_geodesic::PathPoint);
}

void pre_compute_EAR_Oracle_highway_node_T(geodesic::Mesh *mesh, std::vector<int> &highway_node_list,
                                           std::unordered_map<int, double> &distance_highway_node_to_highway_node_map,
                                           std::unordered_map<int, std::vector<geodesic::SurfacePoint>> &path_highway_node_to_highway_node_map,
                                           double &memory_usage)
{
    geodesic::GeodesicAlgorithmExact algorithm(mesh);
    double const distance_limit = geodesic::GEODESIC_INF;
    distance_highway_node_to_highway_node_map.clear();
    path_highway_node_to_highway_node_map.clear();
    int path_highway_node_to_highway_node_size = 0;
    std::vector<geodesic::SurfacePoint> one_source_highway_node_list;
    std::vector<geodesic::SurfacePoint> destinations_highway_node_list;

    for (int i = 0; i < highway_node_list.size(); i++)
    {
        one_source_highway_node_list.clear();
        destinations_highway_node_list.clear();
        one_source_highway_node_list.push_back(geodesic::SurfacePoint(&mesh->vertices()[highway_node_list[i]]));
        for (int j = i; j < highway_node_list.size(); j++)
        {
            destinations_highway_node_list.push_back(geodesic::SurfacePoint(&mesh->vertices()[highway_node_list[j]]));
        }
        algorithm.propagate(one_source_highway_node_list, distance_limit);
        for (int j = i; j < highway_node_list.size(); j++)
        {
            std::vector<geodesic::SurfacePoint> path;
            geodesic::SurfacePoint one_destination(&mesh->vertices()[highway_node_list[j]]);
            algorithm.trace_back(one_destination, path);
            int i_j;
            hash_function_two_keys_to_one_key(highway_node_list.size(), i, j, i_j);
            distance_highway_node_to_highway_node_map[i_j] = length(path);
            path_highway_node_to_highway_node_map[i_j] = path;
            path_highway_node_to_highway_node_size += path.size();
        }
    }
    memory_usage += algorithm.get_memory() + 0.5 * highway_node_list.size() * (highway_node_list.size() - 1) * sizeof(double) + path_highway_node_to_highway_node_size * sizeof(geodesic::SurfacePoint);
}

void poi_to_highway_node_path_C(point_cloud_geodesic::PointCloud *point_cloud, geodesic::Mesh *mesh, int sqrt_num_of_box,
                                std::unordered_map<int, std::unordered_map<int, int>> &highway_node_id_with_box_id_map,
                                std::vector<int> &poi_list, std::unordered_map<int, double> &distance_poi_to_highway_node_map,
                                std::unordered_map<int, std::vector<point_cloud_geodesic::PathPoint>> &path_poi_to_highway_node_map,
                                double &output_size, double &memory_usage)
{
    point_cloud_geodesic::PointCloudGeodesicAlgorithmDijkstra algorithm(point_cloud);
    std::vector<point_cloud_geodesic::PathPoint> one_source_poi_list;
    std::vector<point_cloud_geodesic::PathPoint> destinations_highway_node_list;

    for (int k = 0; k < poi_list.size(); k++)
    {
        one_source_poi_list.clear();
        one_source_poi_list.push_back(point_cloud_geodesic::PathPoint(&point_cloud->pc_points()[poi_list[k]]));

        double box_width = mesh->m_width / sqrt_num_of_box;
        double box_height = mesh->m_height / sqrt_num_of_box;
        int box_id = 0;
        double x_min;
        double x_max;
        double y_min;
        double y_max;

        double poi_x = mesh->vertices()[poi_list[k]].getx();
        double poi_y = mesh->vertices()[poi_list[k]].gety();

        for (int i = 0; i < sqrt_num_of_box; i++)
        {
            for (int j = 0; j < sqrt_num_of_box; j++)
            {
                x_min = mesh->m_xmin + i * box_width;
                x_max = mesh->m_xmin + (i + 1) * box_width;
                y_min = mesh->m_ymin + j * box_height;
                y_max = mesh->m_ymin + (j + 1) * box_height;

                if (poi_x >= x_min && poi_x <= x_max & poi_y >= y_min && poi_y <= y_max)
                {
                    i = sqrt_num_of_box;
                    j = sqrt_num_of_box;
                    continue;
                }
                box_id++;
            }
        }

        std::vector<int> same_box_poi_vertex_id;
        same_box_poi_vertex_id.clear();
        for (int i = 0; i < poi_list.size(); i++)
        {
            if (i == k)
            {
                continue;
            }
            double poi2_x = mesh->vertices()[poi_list[i]].getx();
            double poi2_y = mesh->vertices()[poi_list[i]].gety();
            if (poi2_x >= x_min && poi2_x <= x_max & poi2_y >= y_min && poi2_y <= y_max)
            {
                same_box_poi_vertex_id.push_back(poi_list[i]);
            }
        }

        // add highway node in of current box
        for (auto i : highway_node_id_with_box_id_map[box_id])
        {
            destinations_highway_node_list.push_back(point_cloud_geodesic::PathPoint(&point_cloud->pc_points()[i.first]));
        }
        for (int i = 0; i < same_box_poi_vertex_id.size(); i++)
        {
            destinations_highway_node_list.push_back(point_cloud_geodesic::PathPoint(&point_cloud->pc_points()[same_box_poi_vertex_id[i]]));
        }

        algorithm.propagate(one_source_poi_list, &destinations_highway_node_list);

        for (auto i : highway_node_id_with_box_id_map[box_id])
        {
            point_cloud_geodesic::PathPoint one_dest(&point_cloud->pc_points()[i.first]);
            std::vector<point_cloud_geodesic::PathPoint> path;
            algorithm.trace_back(one_dest, path);

            int x_vertex_id = poi_list[k];
            int y_vertex_id = i.first;
            int x_y_vertex_id;
            if (x_vertex_id <= y_vertex_id)
            {
                hash_function_two_keys_to_one_key(mesh->vertices().size(), x_vertex_id, y_vertex_id, x_y_vertex_id);
            }
            else
            {
                hash_function_two_keys_to_one_key(mesh->vertices().size(), y_vertex_id, x_vertex_id, x_y_vertex_id);
            }
            if (path_poi_to_highway_node_map.count(x_y_vertex_id) == 0)
            {
                path_poi_to_highway_node_map[x_y_vertex_id] = path;
                distance_poi_to_highway_node_map[x_y_vertex_id] = length(path);
                output_size += path.size() * sizeof(point_cloud_geodesic::PathPoint);
                memory_usage += path.size() * sizeof(point_cloud_geodesic::PathPoint);
            }
        }
        for (int i = 0; i < same_box_poi_vertex_id.size(); i++)
        {
            point_cloud_geodesic::PathPoint one_dest(&point_cloud->pc_points()[same_box_poi_vertex_id[i]]);
            std::vector<point_cloud_geodesic::PathPoint> path;
            algorithm.trace_back(one_dest, path);

            int x_vertex_id = poi_list[k];
            int y_vertex_id = same_box_poi_vertex_id[i];
            int x_y_vertex_id;
            if (x_vertex_id <= y_vertex_id)
            {
                hash_function_two_keys_to_one_key(mesh->vertices().size(), x_vertex_id, y_vertex_id, x_y_vertex_id);
            }
            else
            {
                hash_function_two_keys_to_one_key(mesh->vertices().size(), y_vertex_id, x_vertex_id, x_y_vertex_id);
            }
            if (path_poi_to_highway_node_map.count(x_y_vertex_id) == 0)
            {
                path_poi_to_highway_node_map[x_y_vertex_id] = path;
                distance_poi_to_highway_node_map[x_y_vertex_id] = length(path);
                output_size += path.size() * sizeof(point_cloud_geodesic::PathPoint);
                memory_usage += path.size() * sizeof(point_cloud_geodesic::PathPoint);
            }
        }
    }
}

void poi_to_highway_node_path_T(geodesic::Mesh *mesh, int sqrt_num_of_box,
                                std::unordered_map<int, std::unordered_map<int, int>> &highway_node_id_with_box_id_map,
                                std::vector<int> &poi_list, std::unordered_map<int, double> &distance_poi_to_highway_node_map,
                                std::unordered_map<int, std::vector<geodesic::SurfacePoint>> &path_poi_to_highway_node_map,
                                double &output_size, double &memory_usage)
{
    geodesic::GeodesicAlgorithmExact algorithm(mesh);
    std::vector<geodesic::SurfacePoint> one_source_poi_list;
    std::vector<geodesic::SurfacePoint> destinations_highway_node_list;

    for (int k = 0; k < poi_list.size(); k++)
    {
        one_source_poi_list.clear();
        one_source_poi_list.push_back(geodesic::SurfacePoint(&mesh->vertices()[poi_list[k]]));

        double box_width = mesh->m_width / sqrt_num_of_box;
        double box_height = mesh->m_height / sqrt_num_of_box;
        int box_id = 0;
        double x_min;
        double x_max;
        double y_min;
        double y_max;

        double poi_x = mesh->vertices()[poi_list[k]].getx();
        double poi_y = mesh->vertices()[poi_list[k]].gety();

        for (int i = 0; i < sqrt_num_of_box; i++)
        {
            for (int j = 0; j < sqrt_num_of_box; j++)
            {
                x_min = mesh->m_xmin + i * box_width;
                x_max = mesh->m_xmin + (i + 1) * box_width;
                y_min = mesh->m_ymin + j * box_height;
                y_max = mesh->m_ymin + (j + 1) * box_height;

                if (poi_x >= x_min && poi_x <= x_max & poi_y >= y_min && poi_y <= y_max)
                {
                    i = sqrt_num_of_box;
                    j = sqrt_num_of_box;
                    continue;
                }
                box_id++;
            }
        }

        std::vector<int> same_box_poi_vertex_id;
        same_box_poi_vertex_id.clear();
        for (int i = 0; i < poi_list.size(); i++)
        {
            if (i == k)
            {
                continue;
            }
            double poi2_x = mesh->vertices()[poi_list[i]].getx();
            double poi2_y = mesh->vertices()[poi_list[i]].gety();
            if (poi2_x >= x_min && poi2_x <= x_max & poi2_y >= y_min && poi2_y <= y_max)
            {
                same_box_poi_vertex_id.push_back(poi_list[i]);
            }
        }

        for (auto i : highway_node_id_with_box_id_map[box_id])
        {
            destinations_highway_node_list.push_back(geodesic::SurfacePoint(&mesh->vertices()[i.first]));
        }
        for (int i = 0; i < same_box_poi_vertex_id.size(); i++)
        {
            destinations_highway_node_list.push_back(geodesic::SurfacePoint(&mesh->vertices()[same_box_poi_vertex_id[i]]));
        }

        algorithm.propagate(one_source_poi_list, &destinations_highway_node_list);

        for (auto i : highway_node_id_with_box_id_map[box_id])
        {
            geodesic::SurfacePoint one_dest(&mesh->vertices()[i.first]);
            std::vector<geodesic::SurfacePoint> path;
            algorithm.trace_back(one_dest, path);

            int x_vertex_id = poi_list[k];
            int y_vertex_id = i.first;
            int x_y_vertex_id;
            if (x_vertex_id <= y_vertex_id)
            {
                hash_function_two_keys_to_one_key(mesh->vertices().size(), x_vertex_id, y_vertex_id, x_y_vertex_id);
            }
            else
            {
                hash_function_two_keys_to_one_key(mesh->vertices().size(), y_vertex_id, x_vertex_id, x_y_vertex_id);
            }
            if (path_poi_to_highway_node_map.count(x_y_vertex_id) == 0)
            {
                path_poi_to_highway_node_map[x_y_vertex_id] = path;
                distance_poi_to_highway_node_map[x_y_vertex_id] = length(path);
                output_size += path.size() * sizeof(geodesic::SurfacePoint);
                memory_usage += path.size() * sizeof(geodesic::SurfacePoint);
            }
        }

        for (int i = 0; i < same_box_poi_vertex_id.size(); i++)
        {
            geodesic::SurfacePoint one_dest(&mesh->vertices()[same_box_poi_vertex_id[i]]);
            std::vector<geodesic::SurfacePoint> path;
            algorithm.trace_back(one_dest, path);

            int x_vertex_id = poi_list[k];
            int y_vertex_id = same_box_poi_vertex_id[i];
            int x_y_vertex_id;
            if (x_vertex_id <= y_vertex_id)
            {
                hash_function_two_keys_to_one_key(mesh->vertices().size(), x_vertex_id, y_vertex_id, x_y_vertex_id);
            }
            else
            {
                hash_function_two_keys_to_one_key(mesh->vertices().size(), y_vertex_id, x_vertex_id, x_y_vertex_id);
            }
            if (path_poi_to_highway_node_map.count(x_y_vertex_id) == 0)
            {
                path_poi_to_highway_node_map[x_y_vertex_id] = path;
                distance_poi_to_highway_node_map[x_y_vertex_id] = length(path);
                output_size += path.size() * sizeof(geodesic::SurfacePoint);
                memory_usage += path.size() * sizeof(geodesic::SurfacePoint);
            }
        }
    }
}

void three_paths_query_geo_C(point_cloud_geodesic::PointCloud *point_cloud, int geo_tree_node_id,
                             int source_vertex_id,
                             int destination_vertex_id, bool &one_path,
                             std::vector<GeoNode *> &all_poi,
                             std::unordered_map<int, GeoNode *> &geo_node_in_partition_tree_unordered_map,
                             std::unordered_map<int, GeoPair_C *> &geopairs,
                             double &approximate_distance,
                             std::vector<point_cloud_geodesic::PathPoint> &approximate_path)
{
    point_cloud_geodesic::PointCloudGeodesicAlgorithmDijkstra algorithm(point_cloud);

    double approximate_distance_source_short;
    double approximate_distance_destination_short;
    std::vector<point_cloud_geodesic::PathPoint> approximate_path_source_short;
    std::vector<point_cloud_geodesic::PathPoint> approximate_path_destination_short;

    int returned_source_neighbour_index;
    int returned_destination_neighbour_index;

    approximate_distance = query_geo_C(geo_tree_node_id, *geo_node_in_partition_tree_unordered_map[source_vertex_id], *geo_node_in_partition_tree_unordered_map[destination_vertex_id],
                                       returned_source_neighbour_index, returned_destination_neighbour_index, geopairs, approximate_path);
    one_path = true;

    if (destination_vertex_id != returned_destination_neighbour_index)
    {
        int destination_returned_source_neighbour_index;
        int destination_returned_destination_neighbour_index;
        approximate_distance_destination_short = query_geo_C(geo_tree_node_id, *geo_node_in_partition_tree_unordered_map[returned_destination_neighbour_index], *geo_node_in_partition_tree_unordered_map[destination_vertex_id],
                                                             destination_returned_source_neighbour_index, destination_returned_destination_neighbour_index, geopairs, approximate_path_destination_short);
        if (destination_returned_source_neighbour_index != returned_destination_neighbour_index || destination_returned_destination_neighbour_index != destination_vertex_id)
        {
            std::vector<point_cloud_geodesic::PathPoint> one_source_list;
            one_source_list.clear();
            one_source_list.push_back(point_cloud_geodesic::PathPoint(&point_cloud->pc_points()[destination_vertex_id]));
            std::vector<point_cloud_geodesic::PathPoint> one_destination_list;
            one_destination_list.clear();
            one_destination_list.push_back(point_cloud_geodesic::PathPoint(&point_cloud->pc_points()[returned_destination_neighbour_index]));
            algorithm.propagate(one_source_list, &one_destination_list);
            point_cloud_geodesic::PathPoint one_destination(&point_cloud->pc_points()[returned_destination_neighbour_index]);
            algorithm.trace_back(one_destination, approximate_path_destination_short);
            approximate_distance_destination_short = length(approximate_path_destination_short);
        }
        approximate_distance += approximate_distance_destination_short;
        one_path = false;
    }
    if (source_vertex_id != returned_source_neighbour_index)
    {
        int source_returned_source_neighbour_index;
        int source_returned_destination_neighbour_index;
        approximate_distance_source_short = query_geo_C(geo_tree_node_id, *geo_node_in_partition_tree_unordered_map[source_vertex_id], *geo_node_in_partition_tree_unordered_map[returned_source_neighbour_index],
                                                        source_returned_source_neighbour_index, source_returned_destination_neighbour_index, geopairs, approximate_path_source_short);
        if (source_returned_source_neighbour_index != source_vertex_id || source_returned_destination_neighbour_index != returned_source_neighbour_index)
        {
            std::vector<point_cloud_geodesic::PathPoint> one_source_list;
            one_source_list.clear();
            one_source_list.push_back(point_cloud_geodesic::PathPoint(&point_cloud->pc_points()[source_vertex_id]));
            std::vector<point_cloud_geodesic::PathPoint> one_destination_list;
            one_destination_list.clear();
            one_destination_list.push_back(point_cloud_geodesic::PathPoint(&point_cloud->pc_points()[returned_source_neighbour_index]));
            algorithm.propagate(one_source_list, &one_destination_list);
            point_cloud_geodesic::PathPoint one_destination(&point_cloud->pc_points()[returned_source_neighbour_index]);
            algorithm.trace_back(one_destination, approximate_path_source_short);
            approximate_distance_source_short = length(approximate_path_source_short);
        }
        // assert(source_returned_source_neighbour_index == source_vertex_id && source_returned_destination_neighbour_index == returned_source_neighbour_index);
        approximate_distance += approximate_distance_source_short;
        one_path = false;
    }

    if (approximate_path_destination_short.size() > 0)
    {
        if (approximate_path_destination_short[0].getx() == approximate_path[0].getx() &&
            approximate_path_destination_short[0].gety() == approximate_path[0].gety() &&
            approximate_path_destination_short[0].getz() == approximate_path[0].getz())
        {
            std::reverse(approximate_path.begin(), approximate_path.end());
            for (int i = 0; i < approximate_path_destination_short.size(); i++)
            {
                approximate_path.push_back(approximate_path_destination_short[i]);
            }
        }
        else if (approximate_path_destination_short[0].getx() == approximate_path[approximate_path.size() - 1].getx() &&
                 approximate_path_destination_short[0].gety() == approximate_path[approximate_path.size() - 1].gety() &&
                 approximate_path_destination_short[0].getz() == approximate_path[approximate_path.size() - 1].getz())
        {
            for (int i = 0; i < approximate_path_destination_short.size(); i++)
            {
                approximate_path.push_back(approximate_path_destination_short[i]);
            }
        }
        else if (approximate_path_destination_short[approximate_path_destination_short.size() - 1].getx() == approximate_path[0].getx() &&
                 approximate_path_destination_short[approximate_path_destination_short.size() - 1].gety() == approximate_path[0].gety() &&
                 approximate_path_destination_short[approximate_path_destination_short.size() - 1].getz() == approximate_path[0].getz())
        {
            std::reverse(approximate_path.begin(), approximate_path.end());
            for (int i = approximate_path_destination_short.size() - 1; i >= 0; i--)
            {
                approximate_path.push_back(approximate_path_destination_short[i]);
            }
        }
        else if (approximate_path_destination_short[approximate_path_destination_short.size() - 1].getx() == approximate_path[approximate_path.size() - 1].getx() &&
                 approximate_path_destination_short[approximate_path_destination_short.size() - 1].gety() == approximate_path[approximate_path.size() - 1].gety() &&
                 approximate_path_destination_short[approximate_path_destination_short.size() - 1].getz() == approximate_path[approximate_path.size() - 1].getz())
        {
            for (int i = approximate_path_destination_short.size() - 1; i >= 0; i--)
            {
                approximate_path.push_back(approximate_path_destination_short[i]);
            }
        }
        else
        {
            assert(false);
        }
    }

    if (approximate_path_source_short.size() > 0)
    {
        if (approximate_path_source_short[0].getx() == approximate_path[0].getx() &&
            approximate_path_source_short[0].gety() == approximate_path[0].gety() &&
            approximate_path_source_short[0].getz() == approximate_path[0].getz())
        {
            std::reverse(approximate_path.begin(), approximate_path.end());
            for (int i = 0; i < approximate_path_source_short.size(); i++)
            {
                approximate_path.push_back(approximate_path_source_short[i]);
            }
        }
        else if (approximate_path_source_short[0].getx() == approximate_path[approximate_path.size() - 1].getx() &&
                 approximate_path_source_short[0].gety() == approximate_path[approximate_path.size() - 1].gety() &&
                 approximate_path_source_short[0].getz() == approximate_path[approximate_path.size() - 1].getz())
        {
            for (int i = 0; i < approximate_path_source_short.size(); i++)
            {
                approximate_path.push_back(approximate_path_source_short[i]);
            }
        }
        else if (approximate_path_source_short[approximate_path_source_short.size() - 1].getx() == approximate_path[0].getx() &&
                 approximate_path_source_short[approximate_path_source_short.size() - 1].gety() == approximate_path[0].gety() &&
                 approximate_path_source_short[approximate_path_source_short.size() - 1].getz() == approximate_path[0].getz())
        {
            std::reverse(approximate_path.begin(), approximate_path.end());
            for (int i = approximate_path_source_short.size() - 1; i >= 0; i--)
            {
                approximate_path.push_back(approximate_path_source_short[i]);
            }
        }
        else if (approximate_path_source_short[approximate_path_source_short.size() - 1].getx() == approximate_path[approximate_path.size() - 1].getx() &&
                 approximate_path_source_short[approximate_path_source_short.size() - 1].gety() == approximate_path[approximate_path.size() - 1].gety() &&
                 approximate_path_source_short[approximate_path_source_short.size() - 1].getz() == approximate_path[approximate_path.size() - 1].getz())
        {
            for (int i = approximate_path_source_short.size() - 1; i >= 0; i--)
            {
                approximate_path.push_back(approximate_path_source_short[i]);
            }
        }
        else
        {
            assert(false);
        }
    }
}

void three_paths_query_geo_T(geodesic::Mesh *mesh, int geo_tree_node_id,
                             int source_vertex_id,
                             int destination_vertex_id, bool &one_path,
                             std::vector<GeoNode *> &all_poi,
                             std::unordered_map<int, GeoNode *> &geo_node_in_partition_tree_unordered_map,
                             std::unordered_map<int, GeoPair_T *> &geopairs,
                             double &approximate_distance,
                             std::vector<geodesic::SurfacePoint> &approximate_path)
{
    geodesic::GeodesicAlgorithmExact algorithm(mesh);

    double approximate_distance_source_short;
    double approximate_distance_destination_short;
    std::vector<geodesic::SurfacePoint> approximate_path_source_short;
    std::vector<geodesic::SurfacePoint> approximate_path_destination_short;

    int returned_source_neighbour_index;
    int returned_destination_neighbour_index;

    approximate_distance = query_geo_T(geo_tree_node_id, *geo_node_in_partition_tree_unordered_map[source_vertex_id], *geo_node_in_partition_tree_unordered_map[destination_vertex_id],
                                       returned_source_neighbour_index, returned_destination_neighbour_index, geopairs, approximate_path);
    one_path = true;

    if (destination_vertex_id != returned_destination_neighbour_index)
    {
        int destination_returned_source_neighbour_index;
        int destination_returned_destination_neighbour_index;
        approximate_distance_destination_short = query_geo_T(geo_tree_node_id, *geo_node_in_partition_tree_unordered_map[returned_destination_neighbour_index], *geo_node_in_partition_tree_unordered_map[destination_vertex_id],
                                                             destination_returned_source_neighbour_index, destination_returned_destination_neighbour_index, geopairs, approximate_path_destination_short);
        if (destination_returned_source_neighbour_index != returned_destination_neighbour_index || destination_returned_destination_neighbour_index != destination_vertex_id)
        {
            std::vector<geodesic::SurfacePoint> one_source_list;
            one_source_list.clear();
            one_source_list.push_back(geodesic::SurfacePoint(&mesh->vertices()[destination_vertex_id]));
            std::vector<geodesic::SurfacePoint> one_destination_list;
            one_destination_list.clear();
            one_destination_list.push_back(geodesic::SurfacePoint(&mesh->vertices()[returned_destination_neighbour_index]));
            algorithm.propagate(one_source_list, &one_destination_list);
            geodesic::SurfacePoint one_destination(&mesh->vertices()[returned_destination_neighbour_index]);
            algorithm.trace_back(one_destination, approximate_path_destination_short);
            approximate_distance_destination_short = length(approximate_path_destination_short);
        }
        approximate_distance += approximate_distance_destination_short;
        one_path = false;
    }
    if (source_vertex_id != returned_source_neighbour_index)
    {
        int source_returned_source_neighbour_index;
        int source_returned_destination_neighbour_index;
        approximate_distance_source_short = query_geo_T(geo_tree_node_id, *geo_node_in_partition_tree_unordered_map[source_vertex_id], *geo_node_in_partition_tree_unordered_map[returned_source_neighbour_index],
                                                        source_returned_source_neighbour_index, source_returned_destination_neighbour_index, geopairs, approximate_path_source_short);
        if (source_returned_source_neighbour_index != source_vertex_id || source_returned_destination_neighbour_index != returned_source_neighbour_index)
        {
            std::vector<geodesic::SurfacePoint> one_source_list;
            one_source_list.clear();
            one_source_list.push_back(geodesic::SurfacePoint(&mesh->vertices()[source_vertex_id]));
            std::vector<geodesic::SurfacePoint> one_destination_list;
            one_destination_list.clear();
            one_destination_list.push_back(geodesic::SurfacePoint(&mesh->vertices()[returned_source_neighbour_index]));
            algorithm.propagate(one_source_list, &one_destination_list);
            geodesic::SurfacePoint one_destination(&mesh->vertices()[returned_source_neighbour_index]);
            algorithm.trace_back(one_destination, approximate_path_source_short);
            approximate_distance_source_short = length(approximate_path_source_short);
        }
        approximate_distance += approximate_distance_source_short;
        one_path = false;
    }

    if (approximate_path_destination_short.size() > 0)
    {
        if (approximate_path_destination_short[0].getx() == approximate_path[0].getx() &&
            approximate_path_destination_short[0].gety() == approximate_path[0].gety() &&
            approximate_path_destination_short[0].getz() == approximate_path[0].getz())
        {
            std::reverse(approximate_path.begin(), approximate_path.end());
            for (int i = 0; i < approximate_path_destination_short.size(); i++)
            {
                approximate_path.push_back(approximate_path_destination_short[i]);
            }
        }
        else if (approximate_path_destination_short[0].getx() == approximate_path[approximate_path.size() - 1].getx() &&
                 approximate_path_destination_short[0].gety() == approximate_path[approximate_path.size() - 1].gety() &&
                 approximate_path_destination_short[0].getz() == approximate_path[approximate_path.size() - 1].getz())
        {
            for (int i = 0; i < approximate_path_destination_short.size(); i++)
            {
                approximate_path.push_back(approximate_path_destination_short[i]);
            }
        }
        else if (approximate_path_destination_short[approximate_path_destination_short.size() - 1].getx() == approximate_path[0].getx() &&
                 approximate_path_destination_short[approximate_path_destination_short.size() - 1].gety() == approximate_path[0].gety() &&
                 approximate_path_destination_short[approximate_path_destination_short.size() - 1].getz() == approximate_path[0].getz())
        {
            std::reverse(approximate_path.begin(), approximate_path.end());
            for (int i = approximate_path_destination_short.size() - 1; i >= 0; i--)
            {
                approximate_path.push_back(approximate_path_destination_short[i]);
            }
        }
        else if (approximate_path_destination_short[approximate_path_destination_short.size() - 1].getx() == approximate_path[approximate_path.size() - 1].getx() &&
                 approximate_path_destination_short[approximate_path_destination_short.size() - 1].gety() == approximate_path[approximate_path.size() - 1].gety() &&
                 approximate_path_destination_short[approximate_path_destination_short.size() - 1].getz() == approximate_path[approximate_path.size() - 1].getz())
        {
            for (int i = approximate_path_destination_short.size() - 1; i >= 0; i--)
            {
                approximate_path.push_back(approximate_path_destination_short[i]);
            }
        }
        else
        {
            assert(false);
        }
    }

    if (approximate_path_source_short.size() > 0)
    {
        if (approximate_path_source_short[0].getx() == approximate_path[0].getx() &&
            approximate_path_source_short[0].gety() == approximate_path[0].gety() &&
            approximate_path_source_short[0].getz() == approximate_path[0].getz())
        {
            std::reverse(approximate_path.begin(), approximate_path.end());
            for (int i = 0; i < approximate_path_source_short.size(); i++)
            {
                approximate_path.push_back(approximate_path_source_short[i]);
            }
        }
        else if (approximate_path_source_short[0].getx() == approximate_path[approximate_path.size() - 1].getx() &&
                 approximate_path_source_short[0].gety() == approximate_path[approximate_path.size() - 1].gety() &&
                 approximate_path_source_short[0].getz() == approximate_path[approximate_path.size() - 1].getz())
        {
            for (int i = 0; i < approximate_path_source_short.size(); i++)
            {
                approximate_path.push_back(approximate_path_source_short[i]);
            }
        }
        else if (approximate_path_source_short[approximate_path_source_short.size() - 1].getx() == approximate_path[0].getx() &&
                 approximate_path_source_short[approximate_path_source_short.size() - 1].gety() == approximate_path[0].gety() &&
                 approximate_path_source_short[approximate_path_source_short.size() - 1].getz() == approximate_path[0].getz())
        {
            std::reverse(approximate_path.begin(), approximate_path.end());
            for (int i = approximate_path_source_short.size() - 1; i >= 0; i--)
            {
                approximate_path.push_back(approximate_path_source_short[i]);
            }
        }
        else if (approximate_path_source_short[approximate_path_source_short.size() - 1].getx() == approximate_path[approximate_path.size() - 1].getx() &&
                 approximate_path_source_short[approximate_path_source_short.size() - 1].gety() == approximate_path[approximate_path.size() - 1].gety() &&
                 approximate_path_source_short[approximate_path_source_short.size() - 1].getz() == approximate_path[approximate_path.size() - 1].getz())
        {
            for (int i = approximate_path_source_short.size() - 1; i >= 0; i--)
            {
                approximate_path.push_back(approximate_path_source_short[i]);
            }
        }
        else
        {
            assert(false);
        }
    }
}

void EAR_Oracle_query_C(point_cloud_geodesic::PointCloud *point_cloud, geodesic::Mesh *mesh, std::vector<int> &poi_list,
                        int sqrt_num_of_box, int geo_tree_node_id, std::unordered_map<int, std::unordered_map<int, int>> &highway_node_id_with_box_id_map,
                        std::vector<GeoNode *> &all_highway_node, std::unordered_map<int, GeoNode *> &geo_node_in_partition_tree_unordered_map,
                        std::unordered_map<int, GeoPair_C *> &geopairs,
                        std::unordered_map<int, double> &distance_poi_to_highway_node_map, std::unordered_map<int, std::vector<point_cloud_geodesic::PathPoint>> &path_poi_to_highway_node_map,
                        int source_poi_index, int destination_poi_index, double &approximate_distance, std::vector<point_cloud_geodesic::PathPoint> &approximate_path)
{
    bool one_path;

    double box_width = mesh->m_width / sqrt_num_of_box;
    double box_height = mesh->m_height / sqrt_num_of_box;
    int src_box_id = 0;
    int dest_box_id = 0;
    double src_box_x_min, dest_box_x_min;
    double src_box_x_max, dest_box_x_max;
    double src_box_y_min, dest_box_y_min;
    double src_box_y_max, dest_box_y_max;

    double src_x = mesh->vertices()[poi_list[source_poi_index]].getx();
    double src_y = mesh->vertices()[poi_list[source_poi_index]].gety();
    double dest_x = mesh->vertices()[poi_list[destination_poi_index]].getx();
    double dest_y = mesh->vertices()[poi_list[destination_poi_index]].gety();

    // for source poi
    for (int i = 0; i < sqrt_num_of_box; i++)
    {
        for (int j = 0; j < sqrt_num_of_box; j++)
        {
            src_box_x_min = mesh->m_xmin + i * box_width;
            src_box_x_max = mesh->m_xmin + (i + 1) * box_width;
            src_box_y_min = mesh->m_ymin + j * box_height;
            src_box_y_max = mesh->m_ymin + (j + 1) * box_height;

            if (src_x >= src_box_x_min && src_x <= src_box_x_max & src_y >= src_box_y_min && src_y <= src_box_y_max)
            {
                i = sqrt_num_of_box;
                j = sqrt_num_of_box;
                continue;
            }
            src_box_id++;
        }
    }

    for (int i = 0; i < sqrt_num_of_box; i++)
    {
        for (int j = 0; j < sqrt_num_of_box; j++)
        {
            dest_box_x_min = mesh->m_xmin + i * box_width;
            dest_box_x_max = mesh->m_xmin + (i + 1) * box_width;
            dest_box_y_min = mesh->m_ymin + j * box_height;
            dest_box_y_max = mesh->m_ymin + (j + 1) * box_height;

            if (dest_x >= dest_box_x_min && dest_x <= dest_box_x_max & dest_y >= dest_box_y_min && dest_y <= dest_box_y_max)
            {
                i = sqrt_num_of_box;
                j = sqrt_num_of_box;
                continue;
            }
            dest_box_id++;
        }
    }

    if (source_poi_index == destination_poi_index)
    {
        int x_vertex_id = poi_list[source_poi_index];
        approximate_distance = 0;
        approximate_path.clear();
        approximate_path.push_back(point_cloud_geodesic::PathPoint(&point_cloud->pc_points()[x_vertex_id]));
    }
    else
    {
        if (src_box_id == dest_box_id)
        {
            int x_vertex_id = poi_list[source_poi_index];
            int y_vertex_id = poi_list[destination_poi_index];
            int x_y_vertex_id;
            if (x_vertex_id <= y_vertex_id)
            {
                hash_function_two_keys_to_one_key(mesh->vertices().size(), x_vertex_id, y_vertex_id, x_y_vertex_id);
            }
            else
            {
                hash_function_two_keys_to_one_key(mesh->vertices().size(), y_vertex_id, x_vertex_id, x_y_vertex_id);
            }

            assert(path_poi_to_highway_node_map.count(x_y_vertex_id) != 0);
            approximate_distance = distance_poi_to_highway_node_map[x_y_vertex_id];
            approximate_path = path_poi_to_highway_node_map[x_y_vertex_id];
        }
        else
        {
            double min_distance = 1e10;
            std::vector<point_cloud_geodesic::PathPoint> min_src_to_highway_node_path;
            min_src_to_highway_node_path.clear();
            std::vector<point_cloud_geodesic::PathPoint> min_dest_to_highway_node_path;
            min_dest_to_highway_node_path.clear();

            for (auto i : highway_node_id_with_box_id_map[src_box_id])
            {
                for (auto j : highway_node_id_with_box_id_map[dest_box_id])
                {
                    // highway node to highway node
                    double highway_node_to_highway_node_distance = 0;
                    std::vector<point_cloud_geodesic::PathPoint> highway_node_to_highway_node_path;
                    highway_node_to_highway_node_path.clear();
                    std::vector<int> highway_node_to_highway_node_face_sequence_index_list;
                    if (i.first != j.first)
                    {
                        three_paths_query_geo_C(point_cloud, geo_tree_node_id, i.first, j.first, one_path, all_highway_node, geo_node_in_partition_tree_unordered_map, geopairs,
                                                highway_node_to_highway_node_distance, highway_node_to_highway_node_path);
                    }
                    else
                    {
                        highway_node_to_highway_node_path.push_back(point_cloud_geodesic::PathPoint(&point_cloud->pc_points()[i.first]));
                    }

                    double src_to_highway_node_distance = 0;
                    std::vector<point_cloud_geodesic::PathPoint> src_to_highway_node_path;
                    src_to_highway_node_path.clear();

                    int x_vertex_id1 = poi_list[source_poi_index];
                    int y_vertex_id1 = i.first;
                    int x_y_vertex_id1;
                    if (x_vertex_id1 <= y_vertex_id1)
                    {
                        hash_function_two_keys_to_one_key(mesh->vertices().size(), x_vertex_id1, y_vertex_id1, x_y_vertex_id1);
                    }
                    else
                    {
                        hash_function_two_keys_to_one_key(mesh->vertices().size(), y_vertex_id1, x_vertex_id1, x_y_vertex_id1);
                    }
                    assert(path_poi_to_highway_node_map.count(x_y_vertex_id1) != 0);
                    src_to_highway_node_distance = distance_poi_to_highway_node_map[x_y_vertex_id1];
                    src_to_highway_node_path = path_poi_to_highway_node_map[x_y_vertex_id1];
                    if (src_to_highway_node_path.size() == 0)
                    {
                        src_to_highway_node_distance = 0;
                        src_to_highway_node_path.push_back(point_cloud_geodesic::PathPoint(&point_cloud->pc_points()[i.first]));
                    }

                    double dest_to_highway_node_distance = 0;
                    std::vector<point_cloud_geodesic::PathPoint> dest_to_highway_node_path;
                    dest_to_highway_node_path.clear();

                    int x_vertex_id2 = poi_list[destination_poi_index];
                    int y_vertex_id2 = j.first;
                    int x_y_vertex_id2;
                    if (x_vertex_id2 <= y_vertex_id2)
                    {
                        hash_function_two_keys_to_one_key(mesh->vertices().size(), x_vertex_id2, y_vertex_id2, x_y_vertex_id2);
                    }
                    else
                    {
                        hash_function_two_keys_to_one_key(mesh->vertices().size(), y_vertex_id2, x_vertex_id2, x_y_vertex_id2);
                    }
                    assert(path_poi_to_highway_node_map.count(x_y_vertex_id2) != 0);
                    dest_to_highway_node_distance = distance_poi_to_highway_node_map[x_y_vertex_id2];
                    dest_to_highway_node_path = path_poi_to_highway_node_map[x_y_vertex_id2];
                    if (dest_to_highway_node_path.size() == 0)
                    {
                        dest_to_highway_node_distance = 0;
                        dest_to_highway_node_path.push_back(point_cloud_geodesic::PathPoint(&point_cloud->pc_points()[j.first]));
                    }

                    if (min_distance > highway_node_to_highway_node_distance + src_to_highway_node_distance + dest_to_highway_node_distance)
                    {
                        min_distance = highway_node_to_highway_node_distance + src_to_highway_node_distance + dest_to_highway_node_distance;
                        min_src_to_highway_node_path = src_to_highway_node_path;
                        min_dest_to_highway_node_path = dest_to_highway_node_path;
                        approximate_path = highway_node_to_highway_node_path;
                        approximate_distance = min_distance;
                    }
                }
            }

            if (min_dest_to_highway_node_path[0].getx() == approximate_path[0].getx() &&
                min_dest_to_highway_node_path[0].gety() == approximate_path[0].gety() &&
                min_dest_to_highway_node_path[0].getz() == approximate_path[0].getz())
            {
                std::reverse(approximate_path.begin(), approximate_path.end());
                for (int i = 0; i < min_dest_to_highway_node_path.size(); i++)
                {
                    approximate_path.push_back(min_dest_to_highway_node_path[i]);
                }
            }
            else if (min_dest_to_highway_node_path[0].getx() == approximate_path[approximate_path.size() - 1].getx() &&
                     min_dest_to_highway_node_path[0].gety() == approximate_path[approximate_path.size() - 1].gety() &&
                     min_dest_to_highway_node_path[0].getz() == approximate_path[approximate_path.size() - 1].getz())
            {
                for (int i = 0; i < min_dest_to_highway_node_path.size(); i++)
                {
                    approximate_path.push_back(min_dest_to_highway_node_path[i]);
                }
            }
            else if (min_dest_to_highway_node_path[min_dest_to_highway_node_path.size() - 1].getx() == approximate_path[0].getx() &&
                     min_dest_to_highway_node_path[min_dest_to_highway_node_path.size() - 1].gety() == approximate_path[0].gety() &&
                     min_dest_to_highway_node_path[min_dest_to_highway_node_path.size() - 1].getz() == approximate_path[0].getz())
            {
                std::reverse(approximate_path.begin(), approximate_path.end());
                for (int i = min_dest_to_highway_node_path.size() - 1; i >= 0; i--)
                {
                    approximate_path.push_back(min_dest_to_highway_node_path[i]);
                }
            }
            else if (min_dest_to_highway_node_path[min_dest_to_highway_node_path.size() - 1].getx() == approximate_path[approximate_path.size() - 1].getx() &&
                     min_dest_to_highway_node_path[min_dest_to_highway_node_path.size() - 1].gety() == approximate_path[approximate_path.size() - 1].gety() &&
                     min_dest_to_highway_node_path[min_dest_to_highway_node_path.size() - 1].getz() == approximate_path[approximate_path.size() - 1].getz())
            {
                for (int i = min_dest_to_highway_node_path.size() - 1; i >= 0; i--)
                {
                    approximate_path.push_back(min_dest_to_highway_node_path[i]);
                }
            }
            else
            {
                assert(false);
            }

            if (min_src_to_highway_node_path[0].getx() == approximate_path[0].getx() &&
                min_src_to_highway_node_path[0].gety() == approximate_path[0].gety() &&
                min_src_to_highway_node_path[0].getz() == approximate_path[0].getz())
            {
                std::reverse(approximate_path.begin(), approximate_path.end());
                for (int i = 0; i < min_src_to_highway_node_path.size(); i++)
                {
                    approximate_path.push_back(min_src_to_highway_node_path[i]);
                }
            }
            else if (min_src_to_highway_node_path[0].getx() == approximate_path[approximate_path.size() - 1].getx() &&
                     min_src_to_highway_node_path[0].gety() == approximate_path[approximate_path.size() - 1].gety() &&
                     min_src_to_highway_node_path[0].getz() == approximate_path[approximate_path.size() - 1].getz())
            {
                for (int i = 0; i < min_src_to_highway_node_path.size(); i++)
                {
                    approximate_path.push_back(min_src_to_highway_node_path[i]);
                }
            }
            else if (min_src_to_highway_node_path[min_src_to_highway_node_path.size() - 1].getx() == approximate_path[0].getx() &&
                     min_src_to_highway_node_path[min_src_to_highway_node_path.size() - 1].gety() == approximate_path[0].gety() &&
                     min_src_to_highway_node_path[min_src_to_highway_node_path.size() - 1].getz() == approximate_path[0].getz())
            {
                std::reverse(approximate_path.begin(), approximate_path.end());
                for (int i = min_src_to_highway_node_path.size() - 1; i >= 0; i--)
                {
                    approximate_path.push_back(min_src_to_highway_node_path[i]);
                }
            }
            else if (min_src_to_highway_node_path[min_src_to_highway_node_path.size() - 1].getx() == approximate_path[approximate_path.size() - 1].getx() &&
                     min_src_to_highway_node_path[min_src_to_highway_node_path.size() - 1].gety() == approximate_path[approximate_path.size() - 1].gety() &&
                     min_src_to_highway_node_path[min_src_to_highway_node_path.size() - 1].getz() == approximate_path[approximate_path.size() - 1].getz())
            {
                for (int i = min_src_to_highway_node_path.size() - 1; i >= 0; i--)
                {
                    approximate_path.push_back(min_src_to_highway_node_path[i]);
                }
            }
            else
            {
                assert(false);
            }
        }
    }
}

void EAR_Oracle_query_T(geodesic::Mesh *mesh, std::vector<int> &poi_list,
                        int sqrt_num_of_box, int geo_tree_node_id, std::unordered_map<int, std::unordered_map<int, int>> &highway_node_id_with_box_id_map,
                        std::vector<GeoNode *> &all_highway_node, std::unordered_map<int, GeoNode *> &geo_node_in_partition_tree_unordered_map,
                        std::unordered_map<int, GeoPair_T *> &geopairs,
                        std::unordered_map<int, double> &distance_poi_to_highway_node_map, std::unordered_map<int, std::vector<geodesic::SurfacePoint>> &path_poi_to_highway_node_map,
                        int source_poi_index, int destination_poi_index, double &approximate_distance, std::vector<geodesic::SurfacePoint> &approximate_path)
{
    bool one_path;

    double box_width = mesh->m_width / sqrt_num_of_box;
    double box_height = mesh->m_height / sqrt_num_of_box;
    int src_box_id = 0;
    int dest_box_id = 0;
    double src_box_x_min, dest_box_x_min;
    double src_box_x_max, dest_box_x_max;
    double src_box_y_min, dest_box_y_min;
    double src_box_y_max, dest_box_y_max;

    double src_x = mesh->vertices()[poi_list[source_poi_index]].getx();
    double src_y = mesh->vertices()[poi_list[source_poi_index]].gety();
    double dest_x = mesh->vertices()[poi_list[destination_poi_index]].getx();
    double dest_y = mesh->vertices()[poi_list[destination_poi_index]].gety();

    for (int i = 0; i < sqrt_num_of_box; i++)
    {
        for (int j = 0; j < sqrt_num_of_box; j++)
        {
            src_box_x_min = mesh->m_xmin + i * box_width;
            src_box_x_max = mesh->m_xmin + (i + 1) * box_width;
            src_box_y_min = mesh->m_ymin + j * box_height;
            src_box_y_max = mesh->m_ymin + (j + 1) * box_height;

            if (src_x >= src_box_x_min && src_x <= src_box_x_max & src_y >= src_box_y_min && src_y <= src_box_y_max)
            {
                i = sqrt_num_of_box;
                j = sqrt_num_of_box;
                continue;
            }
            src_box_id++;
        }
    }

    for (int i = 0; i < sqrt_num_of_box; i++)
    {
        for (int j = 0; j < sqrt_num_of_box; j++)
        {
            dest_box_x_min = mesh->m_xmin + i * box_width;
            dest_box_x_max = mesh->m_xmin + (i + 1) * box_width;
            dest_box_y_min = mesh->m_ymin + j * box_height;
            dest_box_y_max = mesh->m_ymin + (j + 1) * box_height;

            if (dest_x >= dest_box_x_min && dest_x <= dest_box_x_max & dest_y >= dest_box_y_min && dest_y <= dest_box_y_max)
            {
                i = sqrt_num_of_box;
                j = sqrt_num_of_box;
                continue;
            }
            dest_box_id++;
        }
    }

    if (source_poi_index == destination_poi_index)
    {
        int x_vertex_id = poi_list[source_poi_index];
        approximate_distance = 0;
        approximate_path.clear();
        approximate_path.push_back(geodesic::SurfacePoint(&mesh->vertices()[x_vertex_id]));
    }
    else
    {
        if (src_box_id == dest_box_id)
        {
            int x_vertex_id = poi_list[source_poi_index];
            int y_vertex_id = poi_list[destination_poi_index];
            int x_y_vertex_id;
            if (x_vertex_id <= y_vertex_id)
            {
                hash_function_two_keys_to_one_key(mesh->vertices().size(), x_vertex_id, y_vertex_id, x_y_vertex_id);
            }
            else
            {
                hash_function_two_keys_to_one_key(mesh->vertices().size(), y_vertex_id, x_vertex_id, x_y_vertex_id);
            }

            assert(path_poi_to_highway_node_map.count(x_y_vertex_id) != 0);
            approximate_distance = distance_poi_to_highway_node_map[x_y_vertex_id];
            approximate_path = path_poi_to_highway_node_map[x_y_vertex_id];
        }
        else
        {
            double min_distance = 1e10;
            std::vector<geodesic::SurfacePoint> min_src_to_highway_node_path;
            min_src_to_highway_node_path.clear();
            std::vector<geodesic::SurfacePoint> min_dest_to_highway_node_path;
            min_dest_to_highway_node_path.clear();

            for (auto i : highway_node_id_with_box_id_map[src_box_id])
            {
                for (auto j : highway_node_id_with_box_id_map[dest_box_id])
                {
                    double highway_node_to_highway_node_distance = 0;
                    std::vector<geodesic::SurfacePoint> highway_node_to_highway_node_path;
                    highway_node_to_highway_node_path.clear();
                    std::vector<int> highway_node_to_highway_node_face_sequence_index_list;
                    if (i.first != j.first)
                    {
                        three_paths_query_geo_T(mesh, geo_tree_node_id, i.first, j.first, one_path, all_highway_node, geo_node_in_partition_tree_unordered_map, geopairs,
                                                highway_node_to_highway_node_distance, highway_node_to_highway_node_path);
                    }
                    else
                    {
                        highway_node_to_highway_node_path.push_back(geodesic::SurfacePoint(&mesh->vertices()[i.first]));
                    }

                    double src_to_highway_node_distance = 0;
                    std::vector<geodesic::SurfacePoint> src_to_highway_node_path;
                    src_to_highway_node_path.clear();

                    int x_vertex_id1 = poi_list[source_poi_index];
                    int y_vertex_id1 = i.first;
                    int x_y_vertex_id1;
                    if (x_vertex_id1 <= y_vertex_id1)
                    {
                        hash_function_two_keys_to_one_key(mesh->vertices().size(), x_vertex_id1, y_vertex_id1, x_y_vertex_id1);
                    }
                    else
                    {
                        hash_function_two_keys_to_one_key(mesh->vertices().size(), y_vertex_id1, x_vertex_id1, x_y_vertex_id1);
                    }
                    assert(path_poi_to_highway_node_map.count(x_y_vertex_id1) != 0);
                    src_to_highway_node_distance = distance_poi_to_highway_node_map[x_y_vertex_id1];
                    src_to_highway_node_path = path_poi_to_highway_node_map[x_y_vertex_id1];
                    if (src_to_highway_node_path.size() == 0)
                    {
                        src_to_highway_node_distance = 0;
                        src_to_highway_node_path.push_back(geodesic::SurfacePoint(&mesh->vertices()[i.first]));
                    }

                    double dest_to_highway_node_distance = 0;
                    std::vector<geodesic::SurfacePoint> dest_to_highway_node_path;
                    dest_to_highway_node_path.clear();

                    int x_vertex_id2 = poi_list[destination_poi_index];
                    int y_vertex_id2 = j.first;
                    int x_y_vertex_id2;
                    if (x_vertex_id2 <= y_vertex_id2)
                    {
                        hash_function_two_keys_to_one_key(mesh->vertices().size(), x_vertex_id2, y_vertex_id2, x_y_vertex_id2);
                    }
                    else
                    {
                        hash_function_two_keys_to_one_key(mesh->vertices().size(), y_vertex_id2, x_vertex_id2, x_y_vertex_id2);
                    }
                    assert(path_poi_to_highway_node_map.count(x_y_vertex_id2) != 0);
                    dest_to_highway_node_distance = distance_poi_to_highway_node_map[x_y_vertex_id2];
                    dest_to_highway_node_path = path_poi_to_highway_node_map[x_y_vertex_id2];
                    if (dest_to_highway_node_path.size() == 0)
                    {
                        dest_to_highway_node_distance = 0;
                        dest_to_highway_node_path.push_back(geodesic::SurfacePoint(&mesh->vertices()[j.first]));
                    }

                    if (min_distance > highway_node_to_highway_node_distance + src_to_highway_node_distance + dest_to_highway_node_distance)
                    {
                        min_distance = highway_node_to_highway_node_distance + src_to_highway_node_distance + dest_to_highway_node_distance;
                        min_src_to_highway_node_path = src_to_highway_node_path;
                        min_dest_to_highway_node_path = dest_to_highway_node_path;
                        approximate_path = highway_node_to_highway_node_path;
                        approximate_distance = min_distance;
                    }
                }
            }

            if (min_dest_to_highway_node_path[0].getx() == approximate_path[0].getx() &&
                min_dest_to_highway_node_path[0].gety() == approximate_path[0].gety() &&
                min_dest_to_highway_node_path[0].getz() == approximate_path[0].getz())
            {
                std::reverse(approximate_path.begin(), approximate_path.end());
                for (int i = 0; i < min_dest_to_highway_node_path.size(); i++)
                {
                    approximate_path.push_back(min_dest_to_highway_node_path[i]);
                }
            }
            else if (min_dest_to_highway_node_path[0].getx() == approximate_path[approximate_path.size() - 1].getx() &&
                     min_dest_to_highway_node_path[0].gety() == approximate_path[approximate_path.size() - 1].gety() &&
                     min_dest_to_highway_node_path[0].getz() == approximate_path[approximate_path.size() - 1].getz())
            {
                for (int i = 0; i < min_dest_to_highway_node_path.size(); i++)
                {
                    approximate_path.push_back(min_dest_to_highway_node_path[i]);
                }
            }
            else if (min_dest_to_highway_node_path[min_dest_to_highway_node_path.size() - 1].getx() == approximate_path[0].getx() &&
                     min_dest_to_highway_node_path[min_dest_to_highway_node_path.size() - 1].gety() == approximate_path[0].gety() &&
                     min_dest_to_highway_node_path[min_dest_to_highway_node_path.size() - 1].getz() == approximate_path[0].getz())
            {
                std::reverse(approximate_path.begin(), approximate_path.end());
                for (int i = min_dest_to_highway_node_path.size() - 1; i >= 0; i--)
                {
                    approximate_path.push_back(min_dest_to_highway_node_path[i]);
                }
            }
            else if (min_dest_to_highway_node_path[min_dest_to_highway_node_path.size() - 1].getx() == approximate_path[approximate_path.size() - 1].getx() &&
                     min_dest_to_highway_node_path[min_dest_to_highway_node_path.size() - 1].gety() == approximate_path[approximate_path.size() - 1].gety() &&
                     min_dest_to_highway_node_path[min_dest_to_highway_node_path.size() - 1].getz() == approximate_path[approximate_path.size() - 1].getz())
            {
                for (int i = min_dest_to_highway_node_path.size() - 1; i >= 0; i--)
                {
                    approximate_path.push_back(min_dest_to_highway_node_path[i]);
                }
            }
            else
            {
                assert(false);
            }

            if (min_src_to_highway_node_path[0].getx() == approximate_path[0].getx() &&
                min_src_to_highway_node_path[0].gety() == approximate_path[0].gety() &&
                min_src_to_highway_node_path[0].getz() == approximate_path[0].getz())
            {
                std::reverse(approximate_path.begin(), approximate_path.end());
                for (int i = 0; i < min_src_to_highway_node_path.size(); i++)
                {
                    approximate_path.push_back(min_src_to_highway_node_path[i]);
                }
            }
            else if (min_src_to_highway_node_path[0].getx() == approximate_path[approximate_path.size() - 1].getx() &&
                     min_src_to_highway_node_path[0].gety() == approximate_path[approximate_path.size() - 1].gety() &&
                     min_src_to_highway_node_path[0].getz() == approximate_path[approximate_path.size() - 1].getz())
            {
                for (int i = 0; i < min_src_to_highway_node_path.size(); i++)
                {
                    approximate_path.push_back(min_src_to_highway_node_path[i]);
                }
            }
            else if (min_src_to_highway_node_path[min_src_to_highway_node_path.size() - 1].getx() == approximate_path[0].getx() &&
                     min_src_to_highway_node_path[min_src_to_highway_node_path.size() - 1].gety() == approximate_path[0].gety() &&
                     min_src_to_highway_node_path[min_src_to_highway_node_path.size() - 1].getz() == approximate_path[0].getz())
            {
                std::reverse(approximate_path.begin(), approximate_path.end());
                for (int i = min_src_to_highway_node_path.size() - 1; i >= 0; i--)
                {
                    approximate_path.push_back(min_src_to_highway_node_path[i]);
                }
            }
            else if (min_src_to_highway_node_path[min_src_to_highway_node_path.size() - 1].getx() == approximate_path[approximate_path.size() - 1].getx() &&
                     min_src_to_highway_node_path[min_src_to_highway_node_path.size() - 1].gety() == approximate_path[approximate_path.size() - 1].gety() &&
                     min_src_to_highway_node_path[min_src_to_highway_node_path.size() - 1].getz() == approximate_path[approximate_path.size() - 1].getz())
            {
                for (int i = min_src_to_highway_node_path.size() - 1; i >= 0; i--)
                {
                    approximate_path.push_back(min_src_to_highway_node_path[i]);
                }
            }
            else
            {
                assert(false);
            }
        }
    }
}

void all_poi_knn_or_range_query_EAR_oracle_C(point_cloud_geodesic::PointCloud *point_cloud, geodesic::Mesh *mesh, std::vector<int> &poi_list,
                                             int sqrt_num_of_box, int geo_tree_node_id, std::unordered_map<int, std::unordered_map<int, int>> &highway_node_id_with_box_id_map,
                                             std::vector<GeoNode *> &all_highway_node, std::unordered_map<int, GeoNode *> &geo_node_in_partition_tree_unordered_map,
                                             std::unordered_map<int, GeoPair_C *> &geopairs,
                                             std::unordered_map<int, double> &distance_poi_to_highway_node_map, std::unordered_map<int, std::vector<point_cloud_geodesic::PathPoint>> &path_poi_to_highway_node_map,
                                             int poi_num, int knn_one_range_two, int k_value, double range,
                                             std::vector<std::vector<int>> &all_poi_knn_or_range_list)
{
    std::vector<std::vector<std::pair<double, int>>> poi_to_other_poi_distance_and_index_list;
    poi_to_other_poi_distance_and_index_list.clear();
    std::vector<std::pair<double, int>> one_poi_to_other_poi_distance_and_index_list;
    for (int i = 0; i < poi_num; i++)
    {
        one_poi_to_other_poi_distance_and_index_list.clear();
        for (int j = 0; j < poi_num; j++)
        {
            std::vector<point_cloud_geodesic::PathPoint> path_result;
            double distance_result;
            EAR_Oracle_query_C(point_cloud, mesh, poi_list, sqrt_num_of_box, geo_tree_node_id, highway_node_id_with_box_id_map,
                               all_highway_node, geo_node_in_partition_tree_unordered_map,
                               geopairs, distance_poi_to_highway_node_map, path_poi_to_highway_node_map,
                               i, j, distance_result, path_result);

            one_poi_to_other_poi_distance_and_index_list.push_back(std::make_pair(distance_result, j));
        }
        std::sort(one_poi_to_other_poi_distance_and_index_list.begin(), one_poi_to_other_poi_distance_and_index_list.end());
        poi_to_other_poi_distance_and_index_list.push_back(one_poi_to_other_poi_distance_and_index_list);
    }
    knn_or_range_query(knn_one_range_two, k_value, range, poi_to_other_poi_distance_and_index_list, all_poi_knn_or_range_list);
}

void all_poi_knn_or_range_query_EAR_oracle_T(geodesic::Mesh *mesh, std::vector<int> &poi_list,
                                             int sqrt_num_of_box, int geo_tree_node_id, std::unordered_map<int, std::unordered_map<int, int>> &highway_node_id_with_box_id_map,
                                             std::vector<GeoNode *> &all_highway_node, std::unordered_map<int, GeoNode *> &geo_node_in_partition_tree_unordered_map,
                                             std::unordered_map<int, GeoPair_T *> &geopairs,
                                             std::unordered_map<int, double> &distance_poi_to_highway_node_map, std::unordered_map<int, std::vector<geodesic::SurfacePoint>> &path_poi_to_highway_node_map,
                                             int poi_num, int knn_one_range_two, int k_value, double range,
                                             std::vector<std::vector<int>> &all_poi_knn_or_range_list)
{
    std::vector<std::vector<std::pair<double, int>>> poi_to_other_poi_distance_and_index_list;
    poi_to_other_poi_distance_and_index_list.clear();
    std::vector<std::pair<double, int>> one_poi_to_other_poi_distance_and_index_list;
    for (int i = 0; i < poi_num; i++)
    {
        one_poi_to_other_poi_distance_and_index_list.clear();
        for (int j = 0; j < poi_num; j++)
        {
            std::vector<geodesic::SurfacePoint> path_result;
            double distance_result;
            EAR_Oracle_query_T(mesh, poi_list, sqrt_num_of_box, geo_tree_node_id, highway_node_id_with_box_id_map,
                               all_highway_node, geo_node_in_partition_tree_unordered_map,
                               geopairs, distance_poi_to_highway_node_map, path_poi_to_highway_node_map,
                               i, j, distance_result, path_result);

            one_poi_to_other_poi_distance_and_index_list.push_back(std::make_pair(distance_result, j));
        }
        std::sort(one_poi_to_other_poi_distance_and_index_list.begin(), one_poi_to_other_poi_distance_and_index_list.end());
        poi_to_other_poi_distance_and_index_list.push_back(one_poi_to_other_poi_distance_and_index_list);
    }
    knn_or_range_query(knn_one_range_two, k_value, range, poi_to_other_poi_distance_and_index_list, all_poi_knn_or_range_list);
}
