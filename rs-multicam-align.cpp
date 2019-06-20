// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include "example.hpp"              // Include short list of convenience functions for rendering

#include <map>
#include <vector>

int main(int argc, char * argv[]) try
{
	// Create a simple OpenGL window for rendering:
	window app(1280, 960, "CPP Multi-Camera Example");

	rs2::context                ctx;            // Create librealsense context for managing devices
	rs2::colorizer              colorizer;      // Utility class to convert depth data RGB colorspace
	rs2::threshold_filter		threshold;
	rs2::disparity_transform	to_disparity(true);
	std::vector<rs2::pipeline>  pipelines;
	rs2::align					aligner(RS2_STREAM_COLOR);
	rs2::spatial_filter			spat_filter;
	rs2::temporal_filter		temp_filter;

	colorizer.set_option(RS2_OPTION_COLOR_SCHEME, 9.0);
	colorizer.set_option(RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED, 0);
	colorizer.set_option(RS2_OPTION_MIN_DISTANCE, 0.29);
	colorizer.set_option(RS2_OPTION_MAX_DISTANCE, 10.0);
	threshold.set_option(RS2_OPTION_MIN_DISTANCE, 0.29);
	threshold.set_option(RS2_OPTION_MAX_DISTANCE, 10.0);

	spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2.0f);
	spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5f);
	spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0f);
	temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4f);

	// Start a streaming pipe per each connected device
	for (auto&& dev : ctx.query_devices())
	{
		rs2::pipeline pipe(ctx);
		rs2::config cfg;
		cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
		cfg.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_RGB8, 30);
		cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
		pipe.start(cfg);
		pipelines.emplace_back(pipe);
	}

	// We'll keep track for the last frame of each stream available to make the presentation persistent
	std::map<int, rs2::frame> render_frames;

	// Main app loop
	while (app)
	{
		// Collect the new frames from all the connected devices
		std::vector<rs2::frame> new_frames;
		for (auto &&pipe : pipelines)
		{
			rs2::frameset fs;
			if (pipe.poll_for_frames(&fs))
			{
				fs = aligner.process(fs);
				for (const rs2::frame& f : fs) {
					new_frames.emplace_back(f);
				}
			}
		}


		// Convert the newly-arrived frames to render-firendly format
		for (const auto& frame : new_frames)
		{
			auto processed_frame = threshold.process(frame);
			processed_frame = to_disparity.process(processed_frame);
			processed_frame = spat_filter.process(processed_frame);
			processed_frame = temp_filter.process(processed_frame);
			render_frames[frame.get_profile().unique_id()] = colorizer.process(processed_frame);
		}

		// Present all the collected frames with openGl mosaic
		app.show(render_frames);
	}

	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}
