#include "myalgorithm.h"
#include <vector>
#include <stack>
#include <algorithm>

namespace MyCG
{
    bool ToLeft(const Point_2& p1, const Point_2& p2, const Point_2& p3)
    {
        return (p2.x() - p1.x()) * (p3.y() - p2.y()) - (p2.y() - p1.y()) * (p3.x() - p2.x()) > 0;
    }

    bool InTriangle(const Point_2& p1, const Point_2& p2, const Point_2& p3, const Point_2& p)
    {
        return (ToLeft(p1,p2,p) && ToLeft(p2,p3,p) && ToLeft(p3,p1,p)) || (!ToLeft(p1,p2,p) && !ToLeft(p2,p3,p) && !ToLeft(p3,p1,p));
    }

    bool sort_vertex_by_xy(const Point_2& pp1, const Point_2& pp2)
    {
        if(pp1.x() >= pp2.x())
        {
            if(pp1.x() != pp2.x() || pp1.y() > pp2.y())
                return false; 
            return true;
        }
        return true;
    } 

    bool sort_vertex_by_yx(const Point_2& pp1, const Point_2& pp2)
    {
        if(pp1.y() >= pp2.y())
        {
            if(pp1.y() != pp2.y() || pp1.x() > pp2.x())
                return false; 
            return true;
        }
        return true;
    } 

    int left_than_lowest(const DataPoints_2& rpoints, int begin)
    {
        int index = begin;
        for(int i=begin+1;i<rpoints.size();++i)
        {
            if(rpoints[i].x() < rpoints[index].x())
                index = i;
            else if(rpoints[i].x() == rpoints[index].x() && rpoints[i].y() < rpoints[index].y())
                index = i;
        }
        return index;
    }

    int find_rightest(const DataPoints_2& rpoints, int index)
    {
        int rightest = -1;
        for(int i=0;i<rpoints.size();++i)
        {
            if(i==index) continue;
            if(rightest == -1 || !ToLeft(rpoints[index],rpoints[rightest],rpoints[i]))
                rightest = i;
        }
        return rightest;
    }

    Point_2 Sort_Vertex_by_Angle::p0 = Point_2(-2e32,-2e32);

    void Sort_Vertex_by_Angle::SetP0(const DataPoints_2& rpoints)
    {
        p0 = rpoints[0];
        for(int i=0;i<rpoints.size();++i)
        {
            if(rpoints[i].x() < p0.x())
                p0 = rpoints[i];
            else if(rpoints[i].x() == p0.x() && rpoints[i].y() < p0.y())
                p0 = rpoints[i];
        }
    }

    bool Sort_Vertex_by_Angle::operator()(const Point_2& p1, const Point_2& p2)
    {
        if(p1 == p0)
            return true;
        if(p2 == p0)
            return false;
        if(ToLeft(p0,p1,p2))
            return true;
        return false;
    }

    DataSegments_2 ConvexHull_2::ConvexHull_2_TriMethod(const DataPoints_2& rpoints)
    {
        // 1. Construct all triangles
        std::vector<int>    triangles;
        DataPoints_2        points=rpoints;
        for(int i=0;i<points.size();i++)
            for(int j=i+1;j<points.size();j++)
                for(int k=j+1;k<points.size();k++)
                {
                    triangles.push_back(i);
                    triangles.push_back(j);
                    triangles.push_back(k);
                }

        // 2. Test every point is in a triangle or not
        std::vector<int>    convexhull;
        for(int i=0;i<points.size();i++)
        {
            bool is_in_triangle = false;
            for(int j=0;j<triangles.size();j+=3)
            {
                if(triangles[j] != i && triangles[j+1] != i && triangles[j+2] != i &&
                    InTriangle(points[triangles[j]],points[triangles[j+1]],points[triangles[j+2]],points[i]))
                {
                    is_in_triangle = true;
                    break;
                }
            }
            if(!is_in_triangle)
                convexhull.push_back(i);
        }

        // 3. Construct convex hull
        DataSegments_2 convexhull_lines;
        std::cout << "Total extrem points: " << convexhull.size() << std::endl;
        DataPoints_2    convexhull_points;
        for(int i=0;i<convexhull.size();++i)
            convexhull_points.push_back(points[convexhull[i]]);
        Sort_Vertex_by_Angle::SetP0(convexhull_points);
        std::cout << "P0: " << Sort_Vertex_by_Angle::GetP0() << std::endl;
        std::sort(convexhull_points.begin(),convexhull_points.end(),Sort_Vertex_by_Angle());   
        for(int i=0;i<convexhull.size()-1;i++)
            convexhull_lines.push_back(Segment_2(convexhull_points[i],convexhull_points[i+1]));
        convexhull_lines.push_back(Segment_2(convexhull_points[convexhull.size()-1],convexhull_points[0]));
        return convexhull_lines;
    }

    DataSegments_2 ConvexHull_2::ConvexHull_2_EE(const DataPoints_2& rpoints)
    {
        DataPoints_2        points = rpoints;
        DataSegments_2      convexhull_lines;
        int LTL = left_than_lowest(points);
        std::swap(points[0],points[LTL]);
        for(int i=0;i<points.size();++i)
            for(int j=i+1;j<points.size();++j)
            {
                int flag = 0;
                for(int k=0;k<points.size();++k)
                {
                    if(k == i || k == j)
                        continue;
                    if(ToLeft(points[i],points[j],points[k]))
                        flag ++;
                }
                if(flag==points.size()-2 || flag==0)
                    convexhull_lines.push_back(Segment_2(points[i],points[j]));
            }
        return convexhull_lines;
    }

    DataSegments_2 ConvexHull_2::ConvexHull_2_Jarvis_March(const DataPoints_2& rpoints)
    {
        DataPoints_2 points = rpoints;
        int LTL = left_than_lowest(points);
        DataSegments_2 convexhull_lines;
        int i=LTL;
        do
        {
            int rightest = -1;
            for(int j=0;j<points.size();++j)
            {
                if(i==j) continue;
                if(rightest==-1 || ToLeft(points[i],points[rightest],points[j]))
                    rightest = j;
            }
            convexhull_lines.push_back(Segment_2(points[i],points[rightest]));
            i = rightest;
        }while(LTL!=i);
        return convexhull_lines;
    }

    DataSegments_2 ConvexHull_2::ConvexHull_2_Graham_Scan(const DataPoints_2& rpoints)
    {
        // 1. Preparation work
        DataPoints_2 points = rpoints;
        Sort_Vertex_by_Angle::SetP0(points);
        std::sort(points.begin(),points.end(),Sort_Vertex_by_Angle());
        std::stack<Point_2*>  convexhull_points, temp_stack;
        convexhull_points.push(&points[left_than_lowest(points)]);
        int rightest = find_rightest(points,left_than_lowest(points));
        convexhull_points.push(&points[rightest]);
        for(int i=points.size()-1;i>0;--i)
        {
            if(i==rightest) continue;
            temp_stack.push(&points[i]);
        }

        // 2. Construct convex hull
        while(!temp_stack.empty())
        {
            Point_2* c1 = convexhull_points.top();
            convexhull_points.pop();
            Point_2 *c2 = convexhull_points.top();
            convexhull_points.push(c1);
            Point_2* t1 = temp_stack.top();
            while(!ToLeft(*c2,*c1,*t1))
            {
                convexhull_points.pop();
                c1 = convexhull_points.top();
                convexhull_points.pop();
                c2 = convexhull_points.top();
                convexhull_points.push(c1);
            }
            convexhull_points.push(t1);
            temp_stack.pop();
        }

        DataSegments_2 convexhull_segments;
        auto last = convexhull_points.top();
        while(convexhull_points.size()>1)
        {
            Point_2* tp1 = convexhull_points.top();
            convexhull_points.pop();
            Point_2* tp2 = convexhull_points.top();
            convexhull_segments.push_back(Segment_2(*tp1,*tp2));
        }
        convexhull_segments.push_back(Segment_2(*convexhull_points.top(),*last));
        return convexhull_segments;
    }

    DataSegments_2 ConvexHull_2::ConvexHull_2_Divide_and_Conquer(const DataPoints_2& rpoints)
    {
        return DataSegments_2();
    }
}