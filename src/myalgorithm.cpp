#include "myalgorithm.h"
#include <iostream>
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

    int DAC(const DataPoints_2& rpoints, 
             std::vector<int>& convexhull_points, 
             int begin, int end)
    {
        // 0. Recursive termination condition
        if(end - begin + 1 < 6)
        {
            int LTL = begin;
            for(int i=begin+1;i<=end;++i)
            {
                if(rpoints[i].x() < rpoints[LTL].x())
                    LTL = i;
                else if(rpoints[i].x() == rpoints[LTL].x() && rpoints[i].y() < rpoints[LTL].y())
                    LTL = i;
            }
            int i=LTL, cnt = 0;
            convexhull_points[begin+cnt++] = LTL;
            do
            {
                int rightest = -1;
                for(int j=begin;j<=end;++j)
                {
                    if(i==j) continue;
                    if(rightest==-1 || !ToLeft(rpoints[i],rpoints[rightest],rpoints[j]))
                        rightest = j;
                }
                if(rightest!=LTL) convexhull_points[begin+cnt++] = rightest;
                i = rightest;
            }while(LTL!=i);
            return cnt+begin-1;
        }       
        
        // 1. Divide
        int mid = (begin + end) / 2;
        int l_tail = DAC(rpoints,convexhull_points,begin,mid);
        int r_tail = DAC(rpoints,convexhull_points,mid+1,end);
        // 2. Merge
        // 2.1 find the rightest point in the left part and the leftest point in the right part
        //     find the leftest point in the right part and the rightest point in the left part
        int l_righest = l_tail, r_leftest = mid+1, l_leftest = begin, r_righest = r_tail;
        for(int i=begin;i<=l_tail;++i)
        {   
            if(rpoints[convexhull_points[i]].x() > rpoints[convexhull_points[l_righest]].x())
                l_righest = i;
            else if(rpoints[convexhull_points[i]].x() == rpoints[convexhull_points[l_righest]].x() &&
                    rpoints[convexhull_points[i]].y() >  rpoints[convexhull_points[l_righest]].y())
                l_righest = i;
            if(rpoints[convexhull_points[i]].x() < rpoints[convexhull_points[l_leftest]].x())
                l_leftest = i;
            else if(rpoints[convexhull_points[i]].x() == rpoints[convexhull_points[l_leftest]].x() &&
                    rpoints[convexhull_points[i]].y() <  rpoints[convexhull_points[l_leftest]].y())
                l_leftest = i;
        }
        for(int i=mid+1;i<=r_tail;++i)
        {
            if(rpoints[convexhull_points[i]].x() < rpoints[convexhull_points[r_leftest]].x())
                r_leftest = i;
            else if(rpoints[convexhull_points[i]].x() == rpoints[convexhull_points[r_leftest]].x() &&
                    rpoints[convexhull_points[i]].y() <  rpoints[convexhull_points[r_leftest]].y())
                r_leftest = i;
            if(rpoints[convexhull_points[i]].x() > rpoints[convexhull_points[r_righest]].x())
                r_righest = i;
            else if(rpoints[convexhull_points[i]].x() == rpoints[convexhull_points[r_righest]].x() &&
                    rpoints[convexhull_points[i]].y() >  rpoints[convexhull_points[r_righest]].y())
                r_righest = i;
        }

        // 2.2 Zigzag to find the tangent
        int l_t = -1, r_t = -1, l_b = -1, r_b = -1;
        auto Next_r = [=](int r_index)->int{
            if(r_index == r_tail) return mid + 1;
            return r_index + 1;
        };
        auto Last_r = [=](int r_index)->int{
            if(r_index == mid + 1) return r_tail;
            return r_index - 1;
        };
        auto Next_l = [=](int l_index)->int{
            if(l_index == l_tail) return begin;
            return l_index + 1;
        };
        auto Last_l = [=](int l_index)->int{
            if(l_index == begin) return l_tail;
            return l_index - 1;
        };
        
        int l_r = l_righest, r_l = r_leftest;
        int l_next = Next_l(l_r), l_last = Last_l(l_r),
            r_next = Next_r(r_l), r_last = Last_r(r_l);


        while(l_t == -1 || r_t == -1)
        {
            while(ToLeft(rpoints[convexhull_points[l_r]],rpoints[convexhull_points[r_l]],rpoints[convexhull_points[r_next]]) != false ||
                  ToLeft(rpoints[convexhull_points[l_r]],rpoints[convexhull_points[r_l]],rpoints[convexhull_points[r_last]]) != false)
            {
                r_l = r_last;
                r_next = Next_r(r_l);
                r_last = Last_r(r_l);
            }
            while(ToLeft(rpoints[convexhull_points[r_l]],rpoints[convexhull_points[l_r]],rpoints[convexhull_points[l_next]]) != true || 
                  ToLeft(rpoints[convexhull_points[r_l]],rpoints[convexhull_points[l_r]],rpoints[convexhull_points[l_last]]) != true)
            {
                l_r = l_next;
                l_next = Next_l(l_r);
                l_last = Last_l(l_r);
            }
            if(ToLeft(rpoints[convexhull_points[l_r]],rpoints[convexhull_points[r_l]],rpoints[convexhull_points[r_next]])==false && 
               ToLeft(rpoints[convexhull_points[l_r]],rpoints[convexhull_points[r_l]],rpoints[convexhull_points[r_last]])==false &&
               ToLeft(rpoints[convexhull_points[r_l]],rpoints[convexhull_points[l_r]],rpoints[convexhull_points[l_next]])==true &&
               ToLeft(rpoints[convexhull_points[r_l]],rpoints[convexhull_points[l_r]],rpoints[convexhull_points[l_last]])==true)
            {
                l_t = l_r;
                r_t = r_l;
            }
        }

        l_r = l_righest, r_l = r_leftest;
        r_next = Next_r(r_l), r_last = Last_r(r_l);
        l_next = Next_l(l_r), l_last = Last_l(l_r);
        while(l_b == -1 || r_b == -1)
        {
            while(ToLeft(rpoints[convexhull_points[l_r]],rpoints[convexhull_points[r_l]],rpoints[convexhull_points[r_next]]) != true ||
                  ToLeft(rpoints[convexhull_points[l_r]],rpoints[convexhull_points[r_l]],rpoints[convexhull_points[r_last]]) != true)
            {
                r_l = r_next;
                r_next = Next_r(r_l);
                r_last = Last_r(r_l);
            }
            while(ToLeft(rpoints[convexhull_points[r_l]],rpoints[convexhull_points[l_r]],rpoints[convexhull_points[l_next]]) != false ||
                  ToLeft(rpoints[convexhull_points[r_l]],rpoints[convexhull_points[l_r]],rpoints[convexhull_points[l_last]]) != false)
            {
                l_r = l_last;
                l_next = Next_l(l_r);
                l_last = Last_l(l_r);
            }
            if(ToLeft(rpoints[convexhull_points[l_r]],rpoints[convexhull_points[r_l]],rpoints[convexhull_points[r_next]])==true &&
               ToLeft(rpoints[convexhull_points[l_r]],rpoints[convexhull_points[r_l]],rpoints[convexhull_points[r_last]])==true &&
               ToLeft(rpoints[convexhull_points[r_l]],rpoints[convexhull_points[l_r]],rpoints[convexhull_points[l_next]])==false &&
               ToLeft(rpoints[convexhull_points[r_l]],rpoints[convexhull_points[l_r]],rpoints[convexhull_points[l_last]])==false)
            {
                l_b = l_r;
                r_b = r_l;
            }
        }

        // 2.3 Construct the convex hull
        std::vector<int> convexhull_temp;
        int i=l_t;
        while(i!=l_b)
        {
            convexhull_temp.push_back(convexhull_points[i]);
            i=Next_l(i);
        }
        convexhull_temp.push_back(convexhull_points[i]);
        i = r_b;
        while(i!=r_t)
        {
            convexhull_temp.push_back(convexhull_points[i]);
            i=Next_r(i);
        }     
        convexhull_temp.push_back(convexhull_points[i]);          
        
        for(int i=0;i<convexhull_temp.size();++i)
            convexhull_points[begin+i] = convexhull_temp[i];
        return convexhull_temp.size()-1+begin;
    }

    DataSegments_2 ConvexHull_2::ConvexHull_2_Divide_and_Conquer(const DataPoints_2& rpoints)
    {
        DataPoints_2       points = rpoints;
        std::vector<int>   convexhull_points(points.size());
        std::sort(points.begin(),points.end(),sort_vertex_by_xy);
        int end = DAC(points,convexhull_points,0,points.size()-1);
        DataSegments_2  convexhull_lines;
        for(int i=0;i<end;++i)
            convexhull_lines.push_back(Segment_2(points[convexhull_points[i]],points[convexhull_points[i+1]]));
        convexhull_lines.push_back(Segment_2(points[convexhull_points[end]],points[convexhull_points[0]]));
        return convexhull_lines;
    }
}