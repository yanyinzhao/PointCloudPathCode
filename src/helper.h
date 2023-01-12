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

void knn(int k_value, std::vector<std::vector<std::pair<double, int>>> &poi_to_other_poi_distance_and_index_list,
         std::vector<std::vector<int>> &all_poi_knn_list)
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
                // std::cout << poi_to_other_poi_distance_and_index_list[i][j].second << " ";
            }
        }
        all_poi_knn_list.push_back(one_poi_knn_list);
        // std::cout << std::endl;
    }
}

void calculate_knn_error(std::vector<std::vector<int>> &exact_all_poi_knn_list,
                         std::vector<std::vector<int>> &calculated_all_poi_knn_list,
                         double &knn_error)
{
    int error_count = 0;
    assert(exact_all_poi_knn_list.size() == calculated_all_poi_knn_list.size());
    for (int i = 0; i < exact_all_poi_knn_list.size(); i++)
    {
        assert(exact_all_poi_knn_list[i].size() == calculated_all_poi_knn_list[i].size());
        std::vector<int> a;
        std::vector<int> b;
        for (int j = 0; j < exact_all_poi_knn_list[i].size(); j++)
        {
            a.push_back(exact_all_poi_knn_list[i][j]);
            b.push_back(calculated_all_poi_knn_list[i][j]);
        }
        std::sort(a.begin(), a.end());
        std::sort(b.begin(), b.end());
        for (int j = 0; j < exact_all_poi_knn_list[i].size(); j++)
        {
            if (a[j] != b[j])
            {
                error_count++;
            }
        }
    }
    knn_error = (double)error_count / (double)(exact_all_poi_knn_list.size() * exact_all_poi_knn_list[0].size());
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

void build_level_C(int &geo_tree_node_id, point_cloud_geodesic::PointCloud *point_cloud, int depth,
                   stx::btree<int, GeoNode *> &pois_B_tree, stx::btree<int, GeoNode *> &pois_as_center_each_parent_layer,
                   point_cloud_geodesic::PointCloudGeodesicAlgorithmDijkstra &algorithm)
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
        point_cloud_geodesic::PathPoint source(&point_cloud->pc_points()[n->index]);
        std::vector<point_cloud_geodesic::PathPoint> one_source_list(1, source);
        double const distance_limit = n->radius;
        algorithm.propagate(one_source_list, point_cloud_geodesic::INFIN);
        auto bite = current_parent_covers_but_remained_pois.begin();

        while (bite != current_parent_covers_but_remained_pois.end())
        {
            point_cloud_geodesic::PathPoint p(&point_cloud->pc_points()[(*bite).first]);
            std::vector<point_cloud_geodesic::PathPoint> path;
            algorithm.trace_back(p, path);
            if (length(path) <= distance_limit)
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
            point_cloud_geodesic::PathPoint source(&point_cloud->pc_points()[a->index]);
            std::vector<point_cloud_geodesic::PathPoint> one_source_list(1, source);
            double const distance_limit = b.second->radius;
            algorithm.propagate(one_source_list, point_cloud_geodesic::INFIN);
            auto bite = current_parent_covers_but_remained_pois.begin();

            while (bite != current_parent_covers_but_remained_pois.end())
            {
                point_cloud_geodesic::PathPoint p(&point_cloud->pc_points()[(*bite).first]);
                std::vector<point_cloud_geodesic::PathPoint> path;
                algorithm.trace_back(p, path);
                if (length(path) <= distance_limit)
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
                      point_cloud_geodesic::PointCloudGeodesicAlgorithmDijkstra &algorithm)
{
    int depth = 0;
    pois_as_center_each_parent_layer.clear();
    pois_as_center_each_parent_layer.insert(root_geo.index, &root_geo);
    stx::btree<int, GeoNode *> remained_pois = stx::btree<int, GeoNode *>(pois_B_tree);
    remained_pois.erase(root_geo.index);
    point_cloud_geodesic::PathPoint source(&point_cloud->pc_points()[root_geo.index]);
    std::vector<point_cloud_geodesic::PathPoint> one_source_list(1, source);
    double const distance_limit = root_geo.radius;
    algorithm.propagate(one_source_list, point_cloud_geodesic::INFIN);

    for (stx::btree<int, GeoNode *>::iterator bite = remained_pois.begin(); bite != remained_pois.end(); bite++)
    {
        point_cloud_geodesic::PathPoint p(&point_cloud->pc_points()[(*bite).first]);
        std::vector<point_cloud_geodesic::PathPoint> path;
        algorithm.trace_back(p, path);
        if (length(path) <= distance_limit)
        {
            (*bite).second->set_covered_by(&root_geo);
            (*bite).second->set_covers(&root_geo);
        }
    }

    while (pois_as_center_each_parent_layer.size() != poi_num)
    {
        build_level_C(geo_tree_node_id, point_cloud, depth, pois_B_tree, pois_as_center_each_parent_layer, algorithm);
        depth++;
    }
}

void build_level_TA(int &geo_tree_node_id, geodesic::Mesh *mesh, int depth,
                    stx::btree<int, GeoNode *> &pois_B_tree, stx::btree<int, GeoNode *> &pois_as_center_each_parent_layer,
                    geodesic::GeodesicAlgorithmSubdivision &algorithm, double subdivision_level)
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
        geodesic::SurfacePoint source(&mesh->vertices()[n->index]);
        std::vector<geodesic::SurfacePoint> one_source_list(1, source);
        double const distance_limit = n->radius;
        algorithm.propagate(one_source_list, geodesic::GEODESIC_INF);
        auto bite = current_parent_covers_but_remained_pois.begin();

        while (bite != current_parent_covers_but_remained_pois.end())
        {
            geodesic::SurfacePoint p(&mesh->vertices()[(*bite).first]);
            std::vector<geodesic::SurfacePoint> path;
            algorithm.trace_back(p, path);
            if (subdivision_level == 0)
            {
                modify_path(path);
            }
            if (length(path) <= distance_limit)
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
            geodesic::SurfacePoint source(&mesh->vertices()[a->index]);
            std::vector<geodesic::SurfacePoint> one_source_list(1, source);
            double const distance_limit = b.second->radius;
            algorithm.propagate(one_source_list, geodesic::GEODESIC_INF);
            auto bite = current_parent_covers_but_remained_pois.begin();

            while (bite != current_parent_covers_but_remained_pois.end())
            {
                geodesic::SurfacePoint p(&mesh->vertices()[(*bite).first]);
                std::vector<geodesic::SurfacePoint> path;
                algorithm.trace_back(p, path);
                if (subdivision_level == 0)
                {
                    modify_path(path);
                }
                if (length(path) <= distance_limit)
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

void build_geo_tree_TA(int &geo_tree_node_id, geodesic::Mesh *mesh, GeoNode &root_geo, int poi_num,
                       stx::btree<int, GeoNode *> &pois_B_tree, stx::btree<int, GeoNode *> &pois_as_center_each_parent_layer,
                       geodesic::GeodesicAlgorithmSubdivision &algorithm, double subdivision_level)
{
    int depth = 0;
    pois_as_center_each_parent_layer.clear();
    pois_as_center_each_parent_layer.insert(root_geo.index, &root_geo);
    stx::btree<int, GeoNode *> remained_pois = stx::btree<int, GeoNode *>(pois_B_tree);
    remained_pois.erase(root_geo.index);
    geodesic::SurfacePoint source(&mesh->vertices()[root_geo.index]);
    std::vector<geodesic::SurfacePoint> one_source_list(1, source);
    double const distance_limit = root_geo.radius;
    algorithm.propagate(one_source_list, geodesic::GEODESIC_INF);

    for (stx::btree<int, GeoNode *>::iterator bite = remained_pois.begin(); bite != remained_pois.end(); bite++)
    {
        geodesic::SurfacePoint p(&mesh->vertices()[(*bite).first]);
        std::vector<geodesic::SurfacePoint> path;
        algorithm.trace_back(p, path);
        if (subdivision_level == 0)
        {
            modify_path(path);
        }
        if (length(path) <= distance_limit)
        {
            (*bite).second->set_covered_by(&root_geo);
            (*bite).second->set_covers(&root_geo);
        }
    }

    while (pois_as_center_each_parent_layer.size() != poi_num)
    {
        build_level_TA(geo_tree_node_id, mesh, depth, pois_B_tree, pois_as_center_each_parent_layer, algorithm, subdivision_level);
        depth++;
    }
}

void build_level_TE(int &geo_tree_node_id, geodesic::Mesh *mesh, int depth,
                    stx::btree<int, GeoNode *> &pois_B_tree, stx::btree<int, GeoNode *> &pois_as_center_each_parent_layer,
                    geodesic::GeodesicAlgorithmExact &algorithm)
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
        geodesic::SurfacePoint source(&mesh->vertices()[n->index]);
        std::vector<geodesic::SurfacePoint> one_source_list(1, source);
        double const distance_limit = n->radius;
        algorithm.propagate(one_source_list, geodesic::GEODESIC_INF);
        auto bite = current_parent_covers_but_remained_pois.begin();

        while (bite != current_parent_covers_but_remained_pois.end())
        {
            geodesic::SurfacePoint p(&mesh->vertices()[(*bite).first]);
            std::vector<geodesic::SurfacePoint> path;
            algorithm.trace_back(p, path);
            if (length(path) <= distance_limit)
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
            geodesic::SurfacePoint source(&mesh->vertices()[a->index]);
            std::vector<geodesic::SurfacePoint> one_source_list(1, source);
            double const distance_limit = b.second->radius;
            algorithm.propagate(one_source_list, geodesic::GEODESIC_INF);
            auto bite = current_parent_covers_but_remained_pois.begin();

            while (bite != current_parent_covers_but_remained_pois.end())
            {
                geodesic::SurfacePoint p(&mesh->vertices()[(*bite).first]);
                std::vector<geodesic::SurfacePoint> path;
                algorithm.trace_back(p, path);
                if (length(path) <= distance_limit)
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

void build_geo_tree_TE(int &geo_tree_node_id, geodesic::Mesh *mesh, GeoNode &root_geo, int poi_num,
                       stx::btree<int, GeoNode *> &pois_B_tree, stx::btree<int, GeoNode *> &pois_as_center_each_parent_layer,
                       geodesic::GeodesicAlgorithmExact &algorithm)
{
    int depth = 0;
    pois_as_center_each_parent_layer.clear();
    pois_as_center_each_parent_layer.insert(root_geo.index, &root_geo);
    stx::btree<int, GeoNode *> remained_pois = stx::btree<int, GeoNode *>(pois_B_tree);
    remained_pois.erase(root_geo.index);
    geodesic::SurfacePoint source(&mesh->vertices()[root_geo.index]);
    std::vector<geodesic::SurfacePoint> one_source_list(1, source);
    double const distance_limit = root_geo.radius;
    algorithm.propagate(one_source_list, geodesic::GEODESIC_INF);

    for (stx::btree<int, GeoNode *>::iterator bite = remained_pois.begin(); bite != remained_pois.end(); bite++)
    {
        geodesic::SurfacePoint p(&mesh->vertices()[(*bite).first]);
        std::vector<geodesic::SurfacePoint> path;
        algorithm.trace_back(p, path);
        if (length(path) <= distance_limit)
        {
            (*bite).second->set_covered_by(&root_geo);
            (*bite).second->set_covers(&root_geo);
        }
    }

    while (pois_as_center_each_parent_layer.size() != poi_num)
    {
        build_level_TE(geo_tree_node_id, mesh, depth, pois_B_tree, pois_as_center_each_parent_layer, algorithm);
        depth++;
    }
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

void generate_geo_pair_C(int geo_tree_node_id, int &WSPD_oracle_edge_num,
                         point_cloud_geodesic::PointCloud *point_cloud, GeoNode &x, GeoNode &y,
                         point_cloud_geodesic::PointCloudGeodesicAlgorithmDijkstra &algorithm,
                         double WSPD_oracle_epsilon, std::unordered_map<int, GeoPair_C *> &geopairs,
                         std::unordered_map<int, int> &poi_unordered_map,
                         std::unordered_map<int, int> &geo_pair_unordered_map,
                         std::unordered_map<int, double> &pairwise_distance_unordered_map,
                         std::unordered_map<int, std::vector<point_cloud_geodesic::PathPoint>> &pairwise_path_unordered_map,
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
    hash_function_two_keys_to_one_key(geo_tree_node_id, x_in_geo_node_id, y_in_geo_node_id, x_y_in_geo_node_id);

    if (geo_pair_unordered_map.count(x_y_in_geo_node_id) == 0)
    {
        geo_pair_unordered_map[x_y_in_geo_node_id] = 1; // avoid storing a pair for two times

        assert(poi_unordered_map.count(x.index) != 0 && poi_unordered_map.count(y.index) != 0);
        int x_in_poi_list_for_pairwise_distance_index = poi_unordered_map[x.index];
        int y_in_poi_list_for_pairwise_distance_index = poi_unordered_map[y.index];
        int x_y_in_poi_list_for_pairwise_distance_index;
        if (x_in_poi_list_for_pairwise_distance_index > y_in_poi_list_for_pairwise_distance_index)
        {
            int temp2 = y_in_poi_list_for_pairwise_distance_index;
            y_in_poi_list_for_pairwise_distance_index = x_in_poi_list_for_pairwise_distance_index;
            x_in_poi_list_for_pairwise_distance_index = temp2;
        }
        hash_function_two_keys_to_one_key(poi_unordered_map.size(), x_in_poi_list_for_pairwise_distance_index, y_in_poi_list_for_pairwise_distance_index, x_y_in_poi_list_for_pairwise_distance_index);

        if (pairwise_distance_unordered_map.count(x_y_in_poi_list_for_pairwise_distance_index) == 0)
        {
            pairwise_distance_unordered_map[x_y_in_poi_list_for_pairwise_distance_index] = 0;
            if (&x != &y)
            {
                double const distance_limit = point_cloud_geodesic::INFIN;
                point_cloud_geodesic::PathPoint source(&point_cloud->pc_points()[x.index]);
                std::vector<point_cloud_geodesic::PathPoint> one_source_list(1, source);
                point_cloud_geodesic::PathPoint destination(&point_cloud->pc_points()[y.index]);
                std::vector<point_cloud_geodesic::PathPoint> one_destination_list(1, destination);
                algorithm.propagate(one_source_list, distance_limit);
                std::vector<point_cloud_geodesic::PathPoint> path;
                algorithm.trace_back(destination, path);
                pairwise_path_unordered_map[x_y_in_poi_list_for_pairwise_distance_index] = path;
                pairwise_distance_unordered_map[x_y_in_poi_list_for_pairwise_distance_index] = length(path);
            }
        }

        double distancexy = pairwise_distance_unordered_map[x_y_in_poi_list_for_pairwise_distance_index];
        std::vector<point_cloud_geodesic::PathPoint> pathxy = pairwise_path_unordered_map[x_y_in_poi_list_for_pairwise_distance_index];

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
            hash_function_two_keys_to_one_key(geo_tree_node_id, x_in_geo_node_id_for_geo_pair, y_in_geo_node_id_for_geo_pair, x_y_in_geo_node_id_for_geo_pair);
            geopairs[x_y_in_geo_node_id_for_geo_pair] = nodepair;
        }
        else
        {
            if (x.radius > y.radius)
            {
                for (std::list<GeoNode *>::iterator ite = x.children.begin(); ite != x.children.end(); ite++)
                {
                    generate_geo_pair_C(geo_tree_node_id, WSPD_oracle_edge_num, point_cloud, (**ite), y, algorithm, WSPD_oracle_epsilon, geopairs, poi_unordered_map, geo_pair_unordered_map, pairwise_distance_unordered_map, pairwise_path_unordered_map, pairwise_path_poi_to_poi_size);
                }
            }
            else
            {
                for (std::list<GeoNode *>::iterator jte = y.children.begin(); jte != y.children.end(); jte++)
                {
                    generate_geo_pair_C(geo_tree_node_id, WSPD_oracle_edge_num, point_cloud, x, (**jte), algorithm, WSPD_oracle_epsilon, geopairs, poi_unordered_map, geo_pair_unordered_map, pairwise_distance_unordered_map, pairwise_path_unordered_map, pairwise_path_poi_to_poi_size);
                }
            }
        }
    }
}

void generate_geo_pair_TA(int geo_tree_node_id, int &WSPD_oracle_edge_num,
                          geodesic::Mesh *mesh, GeoNode &x, GeoNode &y,
                          geodesic::GeodesicAlgorithmSubdivision &algorithm, double WSPD_oracle_epsilon,
                          std::unordered_map<int, GeoPair_T *> &geopairs,
                          std::unordered_map<int, int> &poi_unordered_map,
                          std::unordered_map<int, int> &geo_pair_unordered_map,
                          std::unordered_map<int, double> &pairwise_distance_unordered_map,
                          std::unordered_map<int, std::vector<geodesic::SurfacePoint>> &pairwise_path_unordered_map,
                          int &pairwise_path_poi_to_poi_size, double subdivision_level)
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
    hash_function_two_keys_to_one_key(geo_tree_node_id, x_in_geo_node_id, y_in_geo_node_id, x_y_in_geo_node_id);

    if (geo_pair_unordered_map.count(x_y_in_geo_node_id) == 0)
    {
        geo_pair_unordered_map[x_y_in_geo_node_id] = 1; // avoid storing a pair for two times

        assert(poi_unordered_map.count(x.index) != 0 && poi_unordered_map.count(y.index) != 0);
        int x_in_poi_list_for_pairwise_distance_index = poi_unordered_map[x.index];
        int y_in_poi_list_for_pairwise_distance_index = poi_unordered_map[y.index];
        int x_y_in_poi_list_for_pairwise_distance_index;
        if (x_in_poi_list_for_pairwise_distance_index > y_in_poi_list_for_pairwise_distance_index)
        {
            int temp2 = y_in_poi_list_for_pairwise_distance_index;
            y_in_poi_list_for_pairwise_distance_index = x_in_poi_list_for_pairwise_distance_index;
            x_in_poi_list_for_pairwise_distance_index = temp2;
        }
        hash_function_two_keys_to_one_key(poi_unordered_map.size(), x_in_poi_list_for_pairwise_distance_index, y_in_poi_list_for_pairwise_distance_index, x_y_in_poi_list_for_pairwise_distance_index);

        if (pairwise_distance_unordered_map.count(x_y_in_poi_list_for_pairwise_distance_index) == 0)
        {
            pairwise_distance_unordered_map[x_y_in_poi_list_for_pairwise_distance_index] = 0;
            if (&x != &y)
            {
                double const distance_limit = geodesic::GEODESIC_INF;
                geodesic::SurfacePoint source(&mesh->vertices()[x.index]);
                std::vector<geodesic::SurfacePoint> one_source_list(1, source);
                geodesic::SurfacePoint destination(&mesh->vertices()[y.index]);
                std::vector<geodesic::SurfacePoint> one_destination_list(1, destination);
                algorithm.propagate(one_source_list, distance_limit);
                std::vector<geodesic::SurfacePoint> path;
                algorithm.trace_back(destination, path);
                if (subdivision_level == 0)
                {
                    modify_path(path);
                }
                pairwise_path_unordered_map[x_y_in_poi_list_for_pairwise_distance_index] = path;
                pairwise_distance_unordered_map[x_y_in_poi_list_for_pairwise_distance_index] = length(path);
            }
        }

        double distancexy = pairwise_distance_unordered_map[x_y_in_poi_list_for_pairwise_distance_index];
        std::vector<geodesic::SurfacePoint> pathxy = pairwise_path_unordered_map[x_y_in_poi_list_for_pairwise_distance_index];

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
            hash_function_two_keys_to_one_key(geo_tree_node_id, x_in_geo_node_id_for_geo_pair, y_in_geo_node_id_for_geo_pair, x_y_in_geo_node_id_for_geo_pair);
            geopairs[x_y_in_geo_node_id_for_geo_pair] = nodepair;
        }
        else
        {
            if (x.radius > y.radius)
            {
                for (std::list<GeoNode *>::iterator ite = x.children.begin(); ite != x.children.end(); ite++)
                {
                    generate_geo_pair_TA(geo_tree_node_id, WSPD_oracle_edge_num, mesh, (**ite), y, algorithm, WSPD_oracle_epsilon, geopairs, poi_unordered_map, geo_pair_unordered_map, pairwise_distance_unordered_map, pairwise_path_unordered_map, pairwise_path_poi_to_poi_size, subdivision_level);
                }
            }
            else
            {
                for (std::list<GeoNode *>::iterator jte = y.children.begin(); jte != y.children.end(); jte++)
                {
                    generate_geo_pair_TA(geo_tree_node_id, WSPD_oracle_edge_num, mesh, x, (**jte), algorithm, WSPD_oracle_epsilon, geopairs, poi_unordered_map, geo_pair_unordered_map, pairwise_distance_unordered_map, pairwise_path_unordered_map, pairwise_path_poi_to_poi_size, subdivision_level);
                }
            }
        }
    }
}

void generate_geo_pair_TE(int geo_tree_node_id, int &WSPD_oracle_edge_num,
                          geodesic::Mesh *mesh, GeoNode &x, GeoNode &y,
                          geodesic::GeodesicAlgorithmExact &algorithm, double WSPD_oracle_epsilon,
                          std::unordered_map<int, GeoPair_T *> &geopairs,
                          std::unordered_map<int, int> &poi_unordered_map,
                          std::unordered_map<int, int> &geo_pair_unordered_map,
                          std::unordered_map<int, double> &pairwise_distance_unordered_map,
                          std::unordered_map<int, std::vector<geodesic::SurfacePoint>> &pairwise_path_unordered_map,
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
    hash_function_two_keys_to_one_key(geo_tree_node_id, x_in_geo_node_id, y_in_geo_node_id, x_y_in_geo_node_id);

    if (geo_pair_unordered_map.count(x_y_in_geo_node_id) == 0)
    {
        geo_pair_unordered_map[x_y_in_geo_node_id] = 1; // avoid storing a pair for two times

        assert(poi_unordered_map.count(x.index) != 0 && poi_unordered_map.count(y.index) != 0);
        int x_in_poi_list_for_pairwise_distance_index = poi_unordered_map[x.index];
        int y_in_poi_list_for_pairwise_distance_index = poi_unordered_map[y.index];
        int x_y_in_poi_list_for_pairwise_distance_index;
        if (x_in_poi_list_for_pairwise_distance_index > y_in_poi_list_for_pairwise_distance_index)
        {
            int temp2 = y_in_poi_list_for_pairwise_distance_index;
            y_in_poi_list_for_pairwise_distance_index = x_in_poi_list_for_pairwise_distance_index;
            x_in_poi_list_for_pairwise_distance_index = temp2;
        }
        hash_function_two_keys_to_one_key(poi_unordered_map.size(), x_in_poi_list_for_pairwise_distance_index, y_in_poi_list_for_pairwise_distance_index, x_y_in_poi_list_for_pairwise_distance_index);

        if (pairwise_distance_unordered_map.count(x_y_in_poi_list_for_pairwise_distance_index) == 0)
        {
            pairwise_distance_unordered_map[x_y_in_poi_list_for_pairwise_distance_index] = 0;
            if (&x != &y)
            {
                double const distance_limit = geodesic::GEODESIC_INF;
                geodesic::SurfacePoint source(&mesh->vertices()[x.index]);
                std::vector<geodesic::SurfacePoint> one_source_list(1, source);
                geodesic::SurfacePoint destination(&mesh->vertices()[y.index]);
                std::vector<geodesic::SurfacePoint> one_destination_list(1, destination);
                algorithm.propagate(one_source_list, distance_limit);
                std::vector<geodesic::SurfacePoint> path;
                algorithm.trace_back(destination, path);
                pairwise_path_unordered_map[x_y_in_poi_list_for_pairwise_distance_index] = path;
                pairwise_distance_unordered_map[x_y_in_poi_list_for_pairwise_distance_index] = length(path);
            }
        }

        double distancexy = pairwise_distance_unordered_map[x_y_in_poi_list_for_pairwise_distance_index];
        std::vector<geodesic::SurfacePoint> pathxy = pairwise_path_unordered_map[x_y_in_poi_list_for_pairwise_distance_index];

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
            hash_function_two_keys_to_one_key(geo_tree_node_id, x_in_geo_node_id_for_geo_pair, y_in_geo_node_id_for_geo_pair, x_y_in_geo_node_id_for_geo_pair);
            geopairs[x_y_in_geo_node_id_for_geo_pair] = nodepair;
        }
        else
        {
            if (x.radius > y.radius)
            {
                for (std::list<GeoNode *>::iterator ite = x.children.begin(); ite != x.children.end(); ite++)
                {
                    generate_geo_pair_TE(geo_tree_node_id, WSPD_oracle_edge_num, mesh, (**ite), y, algorithm, WSPD_oracle_epsilon, geopairs, poi_unordered_map, geo_pair_unordered_map, pairwise_distance_unordered_map, pairwise_path_unordered_map, pairwise_path_poi_to_poi_size);
                }
            }
            else
            {
                for (std::list<GeoNode *>::iterator jte = y.children.begin(); jte != y.children.end(); jte++)
                {
                    generate_geo_pair_TE(geo_tree_node_id, WSPD_oracle_edge_num, mesh, x, (**jte), algorithm, WSPD_oracle_epsilon, geopairs, poi_unordered_map, geo_pair_unordered_map, pairwise_distance_unordered_map, pairwise_path_unordered_map, pairwise_path_poi_to_poi_size);
                }
            }
        }
    }
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

void all_poi_knn_query_geo_C(int poi_num, int geo_tree_node_id,
                             std::unordered_map<int, GeoNode *> &geo_node_in_partition_tree_unordered_map,
                             std::vector<GeoNode *> &all_poi, std::unordered_map<int, GeoPair_C *> &geopairs,
                             std::unordered_map<int, int> &poi_unordered_map,
                             int k_value, std::vector<std::vector<int>> &all_poi_knn_list)
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
    knn(k_value, poi_to_other_poi_distance_and_index_list, all_poi_knn_list);
}

void all_poi_knn_query_geo_T(int poi_num, int geo_tree_node_id,
                             std::unordered_map<int, GeoNode *> &geo_node_in_partition_tree_unordered_map,
                             std::vector<GeoNode *> &all_poi, std::unordered_map<int, GeoPair_T *> &geopairs,
                             std::unordered_map<int, int> &poi_unordered_map,
                             int k_value, std::vector<std::vector<int>> &all_poi_knn_list)
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
    knn(k_value, poi_to_other_poi_distance_and_index_list, all_poi_knn_list);
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
    // std::cout << x_1 << " " << y_1 << " " << x_2 << " " << y_2 << std::endl;
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
