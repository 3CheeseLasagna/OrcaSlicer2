#include "../ClipperUtils.hpp"
#include "../ShortestPath.hpp"
#include "../Surface.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

#include "FillRectilinear.hpp"  // Rename or create a new header for rectilinear infill

namespace Slic3r {

// This function generates the horizontal or vertical lines for rectilinear infill.
static std::vector<Vec2d> generate_rectilinear_lines(double width, double height, double line_spacing, bool horizontal)
{
    std::vector<Vec2d> points;
    double x_start = 0, y_start = 0;
    
    if (horizontal) {
        for (double y = y_start; y < height; y += line_spacing) {
            points.emplace_back(Vec2d(0, y));          // Start of the line (x=0)
            points.emplace_back(Vec2d(width, y));      // End of the line (x=width)
        }
    } else {
        for (double x = x_start; x < width; x += line_spacing) {
            points.emplace_back(Vec2d(x, 0));          // Start of the line (y=0)
            points.emplace_back(Vec2d(x, height));     // End of the line (y=height)
        }
    }

    return points;
}

// Main function to generate the rectilinear infill pattern
static Polylines make_rectilinear_infill(double width, double height, double line_spacing)
{
    Polylines result;

    // Generate horizontal lines
    auto horizontal_lines = generate_rectilinear_lines(width, height, line_spacing, true);
    Polyline horizontal_polyline;
    for (auto& point : horizontal_lines) {
        horizontal_polyline.points.push_back(point.cast<coord_t>());
    }
    result.push_back(horizontal_polyline);

    // Generate vertical lines
    auto vertical_lines = generate_rectilinear_lines(width, height, line_spacing, false);
    Polyline vertical_polyline;
    for (auto& point : vertical_lines) {
        vertical_polyline.points.push_back(point.cast<coord_t>());
    }
    result.push_back(vertical_polyline);

    return result;
}

// Main function to handle the rectilinear infill
void FillRectilinear::_fill_surface_single(
    const FillParams                &params, 
    unsigned int                     thickness_layers,
    const std::pair<float, Point>   &direction, 
    ExPolygon                        expolygon, 
    Polylines                       &polylines_out)
{
    auto infill_angle = float(this->angle + (CorrectionAngle * 2*M_PI) / 360.);
    if(std::abs(infill_angle) >= EPSILON)
        expolygon.rotate(-infill_angle);

    BoundingBox bb = expolygon.contour.bounding_box();
    // Density adjusted to have a good %of weight.
    double density_adjusted = std::max(0., params.density * DensityAdjust);
    // Distance between the lines in scaled coordinates.
    coord_t distance = coord_t(scale_(this->spacing) / density_adjusted);

    // align bounding box to a multiple of our grid module
    bb.merge(align_to_grid(bb.min, Point(distance, distance)));

    // generate rectilinear pattern
    Polylines polylines = make_rectilinear_infill(
        bb.size()(0),
        bb.size()(1),
        distance);

    // shift the polyline to the grid origin
    for (Polyline &pl : polylines)
        pl.translate(bb.min);

    polylines = intersection_pl(polylines, expolygon);

    if (!polylines.empty()) {
        // Remove very small bits, but be careful to not remove infill lines connecting thin walls!
        const double minlength = scale_(0.8 * this->spacing);
        polylines.erase(
            std::remove_if(polylines.begin(), polylines.end(), [minlength](const Polyline &pl) { return pl.length() < minlength; }),
            polylines.end());
    }

    if (!polylines.empty()) {
        // connect lines
        size_t polylines_out_first_idx = polylines_out.size();
        if (params.dont_connect())
            append(polylines_out, chain_polylines(polylines));
        else
            this->connect_infill(std::move(polylines), expolygon, polylines_out, this->spacing, params);

        // new paths must be rotated back
        if (std::abs(infill_angle) >= EPSILON) {
            for (auto it = polylines_out.begin() + polylines_out_first_idx; it != polylines_out.end(); ++it)
                it->rotate(infill_angle);
        }
    }
}

} // namespace Slic3r
