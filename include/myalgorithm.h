#ifndef _MYALGORITHM_H_
#define _MYALGORITHM_H_

#include "define.h"

namespace MyCG
{
    #define EPS 1e-15
    #define EQZERO(x) (fabs(x) < EPS)
    #define LTZERO(x) (x < -EPS)
    #define GTZERO(x) (x > EPS)
    #define LTEQZERO(x) (x <= EPS)
    #define GTEQZERO(x) (x >= -EPS)

    /**
     * @name Utility functions
     * @brief This function return true if p3 is on the left side of the line p2p1;
     * @details if p1, p2, p3 are collinear, return false;
     * if p3 is on the right side of the line p2p1, return false;
     * Only return true if p3 is on the left side of the line p2p1;
     * */ 
    double crossProduct(const Point_2& p1, const Point_2& p2, const Point_2& p3);

    double dotProduct(const Point_2& p1, const Point_2& p2, const Point_2& p3);

    double crossProduct(const Point_2& p1, const Point_2& p2);

    double Norm(const Point_2& p);
    
    double Norm2(const Point_2& p);

    bool between(const Point_2& p1, const Point_2& p2, const Point_2& p3);

    bool ToLeft(const Point_2& p1, const Point_2& p2, const Point_2& p3);

    bool ToRight(const Point_2& p1, const Point_2& p2, const Point_2& p3);

    bool InTriangle(const Point_2& p1, const Point_2& p2, const Point_2& p3, const Point_2& p);
    
    bool sort_vertex_by_xy(const Point_2& pp1, const Point_2& pp2);

    bool sort_vertex_by_yx(const Point_2& pp1, const Point_2& pp2);

    int left_than_lowest(const DataPoints_2& rpoints, int begin=0);

    int find_rightest(const DataPoints_2& rpoints, int index);

    int DAC(const DataPoints_2& rpoints, std::vector<int>& convexhull_points, int begin, int end);

    class Sort_Vertex_by_Angle
    {
        public:
            Sort_Vertex_by_Angle(){};
            bool operator()(const Point_2& p1, const Point_2& p2);

            static void SetP0(const DataPoints_2& rpoints);
            static Point_2 GetP0(){ return p0; }
        private:
            static Point_2 p0;
    };

    class Sort_Vertex_by_XY
    {
        public:
            bool operator()(const Point_2& p1, const Point_2& p2);
    };

    class Sort_Vertex_by_YX 
    {
        public:
            bool operator()(const Point_2& p1, const Point_2& p2);
    };

    class ConvexHull_2
    {
        public:
            static DataSegments_2 ConvexHull_2_TriMethod(const DataPoints_2& points);
            static DataSegments_2 ConvexHull_2_EE(const DataPoints_2& points);
            static DataSegments_2 ConvexHull_2_Jarvis_March(const DataPoints_2& points);
            static DataSegments_2 ConvexHull_2_Graham_Scan(const DataPoints_2& points);
            static DataSegments_2 ConvexHull_2_Divide_and_Conquer(const DataPoints_2& points);
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

}


#endif // _MYALGORITHM_H_