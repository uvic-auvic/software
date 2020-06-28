#include "Filter.hpp"

/**
 * Must be called in every iteration to work properly.
 * 
 * Takes in the location data of every object found, filters out false positives,
 * then sets x and y to the best location found. 
 */
void Filter::updateFilter(std::vector<cv::Rect> locations)
{
    for(uint8_t i=0; i<locations.size(); i++)
    {
        addPoint(locations.at(i));
        updateBestPoint();
    }

    iteration++;
    if(iteration >= reset_time) {
        removePoint();
        iteration = 0;
    }
}

/**
 * Iterates through points to find the point with the largest n. If n is
 * not greater than 3, x and y are set to 0.
 */
void Filter::updateBestPoint()
{
    uint8_t max = 0;
    uint8_t max_index = 0;
    for(uint8_t i=0; i<points.size(); i++)
    {
        if(points.at(i).n > max) {
            max = points.at(i).n;
            max_index = i;
        }
    }

    if(max > 3) {
        rect.x = points.at(max_index).r.x;
        rect.y = points.at(max_index).r.y;
        rect.width = points.at(max_index).r.width;
        rect.height = points.at(max_index).r.height;
    } 
    else {
        rect.x = 0;
        rect.y = 0;
        rect.width = 0;
        rect.height = 0;
    }
}

/**
 * Iterates through points, adding one to n if the current point is within
 * the set radius. P is then updated to the current point.
 * 
 * If the current point is not within the radius of any point in points,
 * a new index is created for it.  
 */
void Filter::addPoint(cv::Rect point)
{
    for(uint8_t i=0; i<points.size(); i++)
    {
        int curr_x = points.at(i).r.x + points.at(i).r.width/2;
        int curr_y = points.at(i).r.y + points.at(i).r.height/2;
        if(sqrt(pow((rect.x + rect.width/2) - curr_x, 2) + pow((rect.y + rect.height/2) - curr_y, 2)) <= radius) {
            points.at(i).r.x = rect.x;
            points.at(i).r.y = rect.y;
            points.at(i).r.width = rect.width;
            points.at(i).r.height = rect.height;
            points.at(i).n = (points.at(i).n < 10) ? points.at(i).n + 1 : 10;
            return;
        }
    }
    points.push_back({point, 1});
}

/**
 * Iterates through points and removes one from every n in point_data.
 * If n == 0 in any point_data, remove the data from the vector points. 
 */
void Filter::removePoint()
{
    for(uint8_t i=0; i<points.size(); i++)
    {
        points.at(i).n = points.at(i).n - 1;
        if(points.at(i).n == 0) {
            points.erase(points.begin() + i);
            i--;
        }
    }
}