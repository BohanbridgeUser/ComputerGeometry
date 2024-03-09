#ifndef _MYALGORITHM_H_
#define _MYALGORITHM_H_

#include "define.h"
#include "Utility.h"
#include <iostream>
#include <vector>
#include <deque>
#include <stack>
#include <queue>
#include <set>
#include <map>
#include <unordered_map>
#include <algorithm>
#include <iterator>

namespace MyCG
{
    int DAC(const DataPoints_2& rpoints, std::vector<int>& convexhull_points, int begin, int end);

    class ConvexHull_2
    {
        private:
            
        public:
            static DataSegments_2 ConvexHull_2_TriMethod(const DataPoints_2& points);
            static DataSegments_2 ConvexHull_2_EE(const DataPoints_2& points);
            static DataSegments_2 ConvexHull_2_Jarvis_March(const DataPoints_2& points);
            static DataSegments_2 ConvexHull_2_Graham_Scan(const DataPoints_2& points);
            static DataSegments_2 ConvexHull_2_Divide_and_Conquer(const DataPoints_2& points);
            static std::vector<int> ConvexHull_Graham_Scan_Index(const DataPoints_2& rpoints);
            static std::pair<int,int> Find_Top_Tangent( const DataPoints_2& rpoints, 
                                                        const std::vector<int>& convexhull_left,  const std::vector<int>& convexhull_right,
                                                        int begin_l, int end_l, int begin_r, int end_r);
            static std::pair<int,int> Find_Bottom_Tangent(const DataPoints_2& rpoints, 
                                                          const std::vector<int>& convexhull_left,  const std::vector<int>& convexhull_right,
                                                          int begin_l, int end_l, int begin_r, int end_r);
    };

    class Intersection_2
    {
        private:
            enum PointType
            {
                LEFT = 0,
                RIGHT,
                INTERSECTION
            };

            struct Segment_LR;
            struct Point_LR
            {
                Point_2     point;
                PointType   type;
                Segment_LR*  segment;
                Point_LR():point(), type(), segment(nullptr){};
                Point_LR(Point_2& rpoint, PointType rtype, Segment_LR* rsegment = nullptr)
                    :point(rpoint), type(rtype), segment(rsegment){};
            };
            struct Segment_LR 
            {
                Point_LR*  left;
                Point_LR*  right;
                double     height;
                Segment_LR():left(nullptr), right(nullptr), height(){};
                Segment_LR(Point_LR* pleft, Point_LR* pright, double iheight)
                    :left(pleft), right(pright), height(iheight){};

                bool operator<(const Segment_LR& rsegment) const
                {
                    if(height > rsegment.height)
                        return true;
                    else 
                        return false;
                }
            };

            class CMP_pSegment_LR_Y
            {
                public:
                    bool operator()(const Segment_LR* s1, const Segment_LR* s2) const;
            };

            class CMP_Point_LR_X 
            {
                public:
                    bool operator()(const Point_LR& p1, const Point_LR& p2);
            };

            class CMP_Point_LR_Y 
            {
                public:
                    bool operator()(const Point_LR& p1, const Point_LR& p2);
            };

            class CMP_Point_LR_XY
            {
                public:
                    bool operator()(const Point_LR& p1, const Point_LR& p2);
            };

            class CMP_Point_LR_YX
            {
                public:
                    bool operator()(const Point_LR& p1, const Point_LR& p2);
            };

            class CMP_Point_LR_XY_Greater
            {
                public:
                    bool operator()(const Point_LR& p1, const Point_LR& p2);
            };

            class CMP_Segment_Y_Greater
            {
                public:
                    bool operator()(const Segment_2& s1, const Segment_2& s2);
            };

            class CMP_Segment_Y_Less
            {
                public:
                    bool operator()(const Segment_2& s1, const Segment_2& s2);
            };

        public:
            
            static DataSegments_2 Interval(const DataSegments_2& segments);
            static DataPoints_2 Intersection(const DataSegments_2& rsegments);
            static bool Is_Intersection_Segments(const Segment_LR& rsegment1, const Segment_LR& rsegment2);
            static Point_2 Intersection_Segments(const Segment_LR& rsegment1, const Segment_LR& rsegment2);
            static bool Is_Intersection_Segments(const Segment_2& rsegment1, const Segment_2& rsegment2);
            static Point_2 Intersection_Segments(const Segment_2& rsegment1, const Segment_2& rsegment2);
            static bool Is_Monotonechain_Edge_Intersect(const DataPoints_2& chain1, int left1, int right1, 
                                                        const DataPoints_2& chain2, int left2, int right2);
            static std::vector<std::vector<int>> ConvexHull_Intersection(DataPoints_2& convexhull_intersection_points, 
                                                 DataSegments_2& convexhull_intersection_segments,
                                                 const DataPoints_2& rpoints, 
                                                 const DataSegments_2& rsegments);
            
            static void Edge_Chasing(const DataPoints_2& rpoints, DataPoints_2& intersections,
                                     int begin1, int end1, int begin2, int end2);
            
            static void ConvexHull_Edge_Chasing(DataPoints_2& convexhull_intersection_points, 
                                                DataSegments_2& convexhull_intersection_segments,
                                                const DataPoints_2& rpoints, 
                                                const DataSegments_2& rsegments);
            
    };

    class Triangulation
    {
        private:
            struct segment_index
            {
                int begin;
                int end;
            };
            struct Trapezoid{
                segment_index Left;
                segment_index Right;
            };
            class cmp_Trapezoid
            {
                public:
                    static DataPoints_2* p_points;
                    bool operator()(const Trapezoid& rtrapezoid1, const Trapezoid& rtrapezoid2) const;
            };
            static std::vector<std::vector<int>> Monotone_Polygons(DataPoints_2& rpoints,
                                                                   std::vector<std::vector<int>>& ipolygons);
            static std::vector<std::vector<int>> Triangulation_Polygons(const DataPoints_2& rpoints,
                                                                        std::vector<int>& ipolygons);
        public:
            static void Triangulation_Monotone(const DataPoints_2& rpoints, std::vector<DataPoints_2>& triangulations_points);
    
    };

    class Voronoi
    {
        private:
            static Ray_2 Get_Bisector_Ray(const Point_2& rpoint1, const Point_2& rpoint2);
            static void Initialize_Voronoi(Polyhedron& rpolyhedron);
            static bool Is_In_Polygon(const DataPoints_2& rpoints, int index,
                                      const Polyhedron& rpolyhedron, const Facet_handle& rface);
            static Polyhedron trivalVD(const DataPoints_2& rpoints, int begin, int end, const Polyhedron& rpolyhedron);
            static Polyhedron MergeVD(const DataPoints_2& rpoints, Polyhedron& rpolyhedron_left, int begin_l, int end_l,
                                                                   Polyhedron& rpolyhedron_right, int begin_r, int end_r);
            static Polyhedron dacVD(const DataPoints_2& rpoints, int begin, int end, Polyhedron& rpolyhedron);
        public:
            // static void Voronoi_Naive(const DataPoints_2& rpoints, HDS& rhds);
            // static void Voronoi_Incremental(const DataPoints_2& rpoints, HDS& rhds);
            static void Voronoi_Divide_and_Conquer(DataPoints_2& rpoints, Polyhedron& rpolyhedron);
            static void Voronoi_Sweep_Line(const DataPoints_2& rpoints, Polyhedron& rpolyhedron);
    };

}


#endif // _MYALGORITHM_H_