#pragma once

#include <iostream>
#include <fstream>
#include <optional>

#include <pmp/visualization/MeshViewer.h>
#include <pmp/visualization/TrackballViewer.h>
#include <pmp/io/io.h>
#include <imgui.h>

#include <surface_subdivision/config.h>

namespace zone {

class MeshViewer : public pmp::MeshViewer {
public: 
    MeshViewer(const char* title, int width, int height);
private:
    void catmull_clark_subdivision();
    void loop_subdivision();

    void show_notice_loop_subdivision_dialog();
    void show_notice_subdivision_times_dialog();
protected:
    virtual void process_imgui();
private:
    int subdivision_times = 1;
    int subdivision_type = 1;
    std::vector<std::string> models;

    bool show_notice_loop_subdivision = false;
    bool show_notice_subdivision_times = false;
    int total_subdivision_times = 0; // if it surpass 5, it will let your PC stuck
};

}