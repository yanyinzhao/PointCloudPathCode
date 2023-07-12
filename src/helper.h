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
    ~GeoNode(){};
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
                   std::unordered_map<int, GeoPair_C *> &geopairs,
                   std::unordered_map<int, int> &poi_unordered_map,
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
            path_result = geopairs[x_y_in_geo_node_id_for_geo_pair]->path;
            return geopairs[x_y_in_geo_node_id_for_geo_pair]->distance;
        }
    }
    return query_geo_C(geo_tree_node_id, *x.parent, *y.parent, geopairs, poi_unordered_map, path_result);
}

double query_geo_T(int geo_tree_node_id, GeoNode &x, GeoNode &y,
                   std::unordered_map<int, GeoPair_T *> &geopairs,
                   std::unordered_map<int, int> &poi_unordered_map,
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
            path_result = geopairs[x_y_in_geo_node_id_for_geo_pair]->path;
            return geopairs[x_y_in_geo_node_id_for_geo_pair]->distance;
        }
    }
    return query_geo_T(geo_tree_node_id, *x.parent, *y.parent, geopairs, poi_unordered_map, path_result);
}

void all_poi_knn_or_range_query_geo_C(int poi_num, int geo_tree_node_id,
                                      std::unordered_map<int, GeoNode *> &geo_node_in_partition_tree_unordered_map,
                                      std::vector<GeoNode *> &all_poi, std::unordered_map<int, GeoPair_C *> &geopairs,
                                      std::unordered_map<int, int> &poi_unordered_map,
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
            double distance_result = query_geo_C(geo_tree_node_id, *geo_node_in_partition_tree_unordered_map[all_poi[i]->index], *geo_node_in_partition_tree_unordered_map[all_poi[j]->index], geopairs, poi_unordered_map, path_result);

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
                                      std::unordered_map<int, int> &poi_unordered_map,
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
            double distance_result = query_geo_T(geo_tree_node_id, *geo_node_in_partition_tree_unordered_map[all_poi[i]->index], *geo_node_in_partition_tree_unordered_map[all_poi[j]->index], geopairs, poi_unordered_map, path_result);

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

double epslion_to_subdivision_level(double epsilon)
{
    assert(epsilon > 0 && epsilon <= 1);
    double subdivision_level = 0;
    if (epsilon > 0 && epsilon <= 0.05)
    {
        subdivision_level = floor(7 / (10 * epsilon)) - 8;
    }
    else if (epsilon > 0.05 && epsilon <= 0.1)
    {
        subdivision_level = floor(7 / (10 * epsilon)) - 2;
    }
    else if (epsilon > 0.1 && epsilon <= 0.25)
    {
        subdivision_level = floor(1 / epsilon) - 1;
    }
    else if (epsilon > 0.25 && epsilon < 1)
    {
        subdivision_level = floor(1 / epsilon);
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

void pre_compute_Vertex_FaceAppr(int poi_num, geodesic::Mesh *mesh, std::vector<int> &poi_list, double epsilon,
                                 bool pass_point_and_not_pass_terrain, std::unordered_map<int, double> &pairwise_distance_poi_to_poi_map,
                                 std::unordered_map<int, std::vector<geodesic::SurfacePoint>> &pairwise_path_poi_to_poi_map,
                                 double &memory_usage)
{
    double subdivision_level = 0;
    if (!pass_point_and_not_pass_terrain)
    {
        subdivision_level = epslion_to_subdivision_level(epsilon);
    }
    geodesic::GeodesicAlgorithmSubdivision algorithm(mesh, subdivision_level);
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
            if (subdivision_level == 0)
            {
                modify_path(path);
            }
            int i_j;
            hash_function_two_keys_to_one_key(poi_num, i, j, i_j);
            pairwise_distance_poi_to_poi_map[i_j] = length(path);
            pairwise_path_poi_to_poi_map[i_j] = path;
            pairwise_path_poi_to_poi_size += path.size();
        }
    }
    memory_usage += algorithm.get_memory() + 0.5 * poi_num * (poi_num - 1) * sizeof(double) + pairwise_path_poi_to_poi_size * sizeof(geodesic::SurfacePoint);
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
                         double WSPD_oracle_epsilon, std::unordered_map<int, GeoPair_C *> &geopairs,
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
        geo_pair_unordered_map[x_y_in_geo_node_id] = 1; // avoid storing a pair for two times

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
        if (distancexy >= (2.0 / WSPD_oracle_epsilon + 2.0) * max(x.radius, y.radius))
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
                    generate_geo_pair_C(geo_tree_node_id, WSPD_oracle_edge_num, point_cloud, (**ite), y, WSPD_oracle_epsilon, geopairs, poi_unordered_map, geo_pair_unordered_map, pre_pairwise_distance_poi_to_poi_map, pre_pairwise_path_poi_to_poi_map, pairwise_path_poi_to_poi_size);
                }
            }
            else
            {
                for (std::list<GeoNode *>::iterator jte = y.children.begin(); jte != y.children.end(); jte++)
                {
                    generate_geo_pair_C(geo_tree_node_id, WSPD_oracle_edge_num, point_cloud, x, (**jte), WSPD_oracle_epsilon, geopairs, poi_unordered_map, geo_pair_unordered_map, pre_pairwise_distance_poi_to_poi_map, pre_pairwise_path_poi_to_poi_map, pairwise_path_poi_to_poi_size);
                }
            }
        }
    }
}

void generate_geo_pair_T(int geo_tree_node_id, int &WSPD_oracle_edge_num,
                         geodesic::Mesh *mesh, GeoNode &x, GeoNode &y,
                         double WSPD_oracle_epsilon,
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
        geo_pair_unordered_map[x_y_in_geo_node_id] = 1; // avoid storing a pair for two times

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
        if (distancexy >= (2.0 / WSPD_oracle_epsilon + 2.0) * max(x.radius, y.radius))
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
                    generate_geo_pair_T(geo_tree_node_id, WSPD_oracle_edge_num, mesh, (**ite), y, WSPD_oracle_epsilon, geopairs, poi_unordered_map, geo_pair_unordered_map, pre_pairwise_distance_poi_to_poi_map, pre_pairwise_path_poi_to_poi_map, pairwise_path_poi_to_poi_size);
                }
            }
            else
            {
                for (std::list<GeoNode *>::iterator jte = y.children.begin(); jte != y.children.end(); jte++)
                {
                    generate_geo_pair_T(geo_tree_node_id, WSPD_oracle_edge_num, mesh, x, (**jte), WSPD_oracle_epsilon, geopairs, poi_unordered_map, geo_pair_unordered_map, pre_pairwise_distance_poi_to_poi_map, pre_pairwise_path_poi_to_poi_map, pairwise_path_poi_to_poi_size);
                }
            }
        }
    }
}