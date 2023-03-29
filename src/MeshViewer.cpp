#include <surface_subdivision/MeshViewer.h>

namespace zone {

MeshViewer::MeshViewer(const char* title, int width, int height)
    : pmp::MeshViewer(title, width, height) {
    set_draw_mode("Hidden Line");
    std::filesystem::path model_path(zone::current_path + "/model");
    for (auto& file : std::filesystem::directory_iterator(model_path)) {
        models.push_back(file.path().stem().string());
    }
    std::string first = zone::current_path + "/model/" + models.front() + ".obj";
    load_mesh(first.c_str());
}

void MeshViewer::process_imgui() {
    pmp::MeshViewer::process_imgui();
    ImGui::Spacing();
    if (ImGui::CollapsingHeader("model", ImGuiTreeNodeFlags_DefaultOpen)) {
        static int item_current_idx = 0;
        const char* combo_preview_value = (models[item_current_idx]).c_str();
        if (ImGui::BeginCombo("models", combo_preview_value))
        {
            for (int n = 0; n < models.size(); n++)
            {
                const bool is_selected = (item_current_idx == n);
                if (ImGui::Selectable((models[n]).c_str(), is_selected)) {
                    std::string filename = zone::current_path + "/model/" + models[n] + ".obj";
                    this->load_mesh(filename.c_str());
                    total_subdivision_times = 0;
                    item_current_idx = n;
                }

                if (is_selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        if (ImGui::Button("reload mesh")) {
            total_subdivision_times = 0;
            load_mesh(filename_.c_str());
        }
    }

    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Subdivision", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::RadioButton("loop subdivision (valid only for triangle mesh)", &subdivision_type, 0);
        ImGui::RadioButton("catmull clark subdivision", &subdivision_type, 1);
        ImGui::SliderInt("subdivision times", &subdivision_times, 1, 5);
        ImGui::Text("total subdivision times: %d", total_subdivision_times);
        show_notice_loop_subdivision_dialog();
        show_notice_subdivision_times_dialog();
        if (ImGui::Button("execute")) {
            if (total_subdivision_times + subdivision_times > 5) {
                show_notice_subdivision_times = true;
                return ;
            }
            // subdivision
            total_subdivision_times += subdivision_times;
            switch (subdivision_type) {
            case 0: 
                if (mesh_.is_triangle_mesh()) {
                    for (int i = 0; i < subdivision_times; i++) {
                        loop_subdivision();
                    }
                } else {
                    show_notice_loop_subdivision = true;
                }
                break;
            case 1: 
                for (int i = 0; i < subdivision_times; i++) {
                    catmull_clark_subdivision();
                }
                break;
            }
            update_mesh();
        }
    }

    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Save as", ImGuiTreeNodeFlags_DefaultOpen)) {
        static char filename[128] = "save.obj";
        ImGui::InputText("filename", filename, IM_ARRAYSIZE(filename));
        if (ImGui::Button("save")) {
            // save mesh to file
            std::filesystem::path p(zone::current_path + "/model/" + filename);
            pmp::write(mesh_, p);
        }
    }
}

void MeshViewer::catmull_clark_subdivision() {
    pmp::SurfaceMesh new_mesh;
    pmp::SurfaceMesh mesh;
    mesh.assign(mesh_);
    auto face_p = mesh.add_face_property<pmp::Point>("f:face_p");
    auto edge_p = mesh.add_edge_property<pmp::Point>("e:edge_p");
    auto opt_p  = mesh.add_vertex_property<std::optional<pmp::Vertex>>("v:opt_p", std::nullopt);
    // face point
    for (auto face : mesh.faces()) {
        pmp::Point fp{0.0f, 0.0f, 0.0f};
        int n = 0; // number of vertices 
        for (auto v : mesh.vertices(face)) {
            auto p = mesh.position(v);
            fp += p;
            n++;
        }
        fp /= (float)n;
        face_p[face] = fp;
    }

    // edge point
    for (auto edge : mesh.edges()) {
        if (mesh.is_boundary(edge)) {
            auto p0 = mesh.position(mesh.vertex(edge, 0));
            auto p1 = mesh.position(mesh.vertex(edge, 1));
            edge_p[edge] = (p0 + p1) * 0.5f;
        } else {
            auto p0 = mesh.position(mesh.vertex(edge, 0));
            auto p1 = mesh.position(mesh.vertex(edge, 1));
            auto p2 = face_p[mesh.face(edge, 0)];
            auto p3 = face_p[mesh.face(edge, 1)];
            edge_p[edge] = (p0 + p1 + p2 + p3) * 0.25f;
        }
        
    }

    // update vertex point
    auto vpoint = mesh.get_vertex_property<pmp::Point>("v:point");
    for (auto vertex : mesh.vertices()) {
        if (mesh.is_boundary(vertex)) {
            auto h1 = mesh.halfedge(vertex);
            auto h0 = mesh.prev_halfedge(h1);
            pmp::Point np = vpoint[vertex];
            np *= 6.0f;
            np += vpoint[mesh.to_vertex(h1)];
            np += vpoint[mesh.from_vertex(h0)];
            np *= 0.125f;
            vpoint[vertex] = np;
        } else {
            pmp::Point np{0.0f, 0.0f, 0.0f};
            pmp::Point fp{0.0f, 0.0f, 0.0f}, ep{0.0f, 0.0f, 0.0f};
            float n = 0.0f;
            for (auto face : mesh.faces(vertex)) {
                fp += face_p[face];
                n += 1.0f;
            }
            fp /= n;
            for (auto he : mesh.halfedges(vertex)) {
                ep += edge_p[mesh.edge(he)] * 2.0f;
            }
            ep /= n;
            np = (n - 3.0f) * mesh.position(vertex) + fp + ep;
            np /= n;
            vpoint[vertex] = np;
        }
    }

    auto new_vh = mesh.add_edge_property<pmp::Vertex>("e:new_vh");
    for (auto edge : mesh.edges()) {
        new_vh[edge] = mesh.add_vertex(edge_p[edge]);
    }
    
    // split face
    for (auto face : mesh.faces()) {
        std::vector<pmp::Vertex> vhs;
        std::vector<pmp::Vertex> vhs_;
        auto vf = mesh.add_vertex(face_p[face]);
        for (auto he : mesh.halfedges(face)) {
            vhs.clear();
            vhs_.clear();
            auto ve1 = new_vh[mesh.edge(he)];
            auto vhe = mesh.to_vertex(he);
            auto next = mesh.next_halfedge(he);
            auto ve2 = new_vh[mesh.edge(next)];
            vhs.push_back(vf);
            vhs.push_back(ve1);
            vhs.push_back(vhe);
            vhs.push_back(ve2);
            for (auto vh : vhs) {
                if (!opt_p[vh]) {
                    opt_p[vh] = new_mesh.add_vertex(mesh.position(vh));
                }
                vhs_.push_back(opt_p[vh].value());
            }
            new_mesh.add_face(vhs_);
        }
    }
    mesh.clear();
    mesh_.assign(new_mesh);
}

void MeshViewer::loop_subdivision() {
    if (!mesh_.is_triangle_mesh()) {
        throw pmp::InvalidInputException("Not a triagnle mesh");
    }
    
    pmp::SurfaceMesh mesh = mesh_;
    pmp::SurfaceMesh new_mesh;
    auto edge_p = mesh.add_edge_property<pmp::Point>("edge_p");
    
    // compute edge point 
    for (auto& edge : mesh.edges()) {
        if (mesh.is_boundary(edge)) {
            auto v0 = mesh.vertex(edge, 0);
            auto v1 = mesh.vertex(edge, 1);
            auto p0 = mesh.position(v0);
            auto p1 = mesh.position(v1);
            edge_p[edge] = (p0 + p1) / 2.0f;
        } else {
            auto v0 = mesh.vertex(edge, 0);
            auto v1 = mesh.vertex(edge, 1);
            auto p0 = mesh.position(v0);
            auto p1 = mesh.position(v1);

            auto h0 = mesh.halfedge(edge, 0);
            auto h1 = mesh.halfedge(edge, 1);
            auto f0 = mesh.face(h0);
            auto f1 = mesh.face(h1);

            pmp::Point p2, p3;
            for (auto v : mesh.vertices(f0)) {
                auto p = mesh.position(v);
                if (p != p0 && p != p1) {
                    p2 = p;
                    break;
                }
            }
            for (auto v : mesh.vertices(f1)) {
                auto p = mesh.position(v);
                if (p != p0 && p != p1) {
                    p3 = p;
                    break;
                }
            }
            edge_p[edge] = 0.375f * (p0 + p1) + 0.125f * (p2 + p3);
        }
    }

    // update origin point
    auto vpoint = mesh.get_vertex_property<pmp::Point>("v:point");
    for (auto& v : mesh.vertices()) {
        if (mesh.is_boundary(v)) {
            pmp::Point np{0.0f, 0.0f, 0.0f};
            for (auto vv : mesh.vertices(v)) {
                if (!mesh.is_boundary(vv))
                    continue;
                np += mesh.position(vv) * 0.125f;
            }
            np += 0.75f * mesh.position(v);
            vpoint[v] = np;
        } else {
            auto O = mesh.position(v); // origin
            float n = 0.0f;
            pmp::Point Q{0.0f, 0.0f, 0.0f};
            for (auto& vv : mesh.vertices(v)) {
                Q += mesh.position(vv);
                n += 1.0f;
            }
            Q /= n;
            float alpha = 0.625f - (0.375f + 0.25f * std::cosf(2 * EIGEN_PI / n)) * (0.375f + 0.25f * std::cosf(2 * EIGEN_PI / n));
            vpoint[v] = (1.0f - alpha) * O + alpha * Q;
        }
    }

    // update the topology of mesh
    /**
     *           v0
     *        /     \
     *      e0      e1
     *     /   \   /   \ 
     *  v2 ---- e2 ---- v1
    */
    auto v_opt = mesh.add_vertex_property<std::optional<pmp::Vertex>>("v:v_opt", std::nullopt);
    auto e_opt = mesh.add_edge_property<std::optional<pmp::Vertex>>("e:e_opt", std::nullopt);
    for (auto face : mesh.faces()) {
        // pmp::Vertex
        std::vector<pmp::Vertex> vs_;
        std::vector<pmp::Edge>   es_;
        for (auto he : mesh.halfedges(face)) {
            vs_.push_back(mesh.to_vertex(he));
            es_.push_back(mesh.edge(he));
        }
        pmp::Vertex v0, v1, v2, e0, e1, e2;

        if (v_opt[vs_[0]].has_value()) {
            v0 = v_opt[vs_[0]].value();
        } else {
            v0 = new_mesh.add_vertex(mesh.position(vs_[0]));
            v_opt[vs_[0]] = v0;
        }

        if (v_opt[vs_[1]].has_value()) {
            v1 = v_opt[vs_[1]].value();
        } else {
            v1 = new_mesh.add_vertex(mesh.position(vs_[1]));
            v_opt[vs_[1]] = v1;
        }

        if (v_opt[vs_[2]].has_value()) {
            v2 = v_opt[vs_[2]].value();
        } else {
            v2 = new_mesh.add_vertex(mesh.position(vs_[2]));
            v_opt[vs_[2]] = v2;
        }

        if (e_opt[es_[0]].has_value()) {
            e0 = e_opt[es_[0]].value();
        } else {
            e0 = new_mesh.add_vertex(edge_p[es_[0]]);
            e_opt[es_[0]] = e0;
        }

        if (e_opt[es_[1]].has_value()) {
            e1 = e_opt[es_[1]].value();
        } else {
            e1 = new_mesh.add_vertex(edge_p[es_[1]]);
            e_opt[es_[1]] = e1;
        }

        if (e_opt[es_[2]].has_value()) {
            e2 = e_opt[es_[2]].value();
        } else {
            e2 = new_mesh.add_vertex(edge_p[es_[2]]);
            e_opt[es_[2]] = e2;
        }

        // split a triangle into four triangle
        new_mesh.add_triangle(v0, e1, e0);
        new_mesh.add_triangle(e1, v1, e2);
        new_mesh.add_triangle(e0, e1, e2);
        new_mesh.add_triangle(e0, e2, v2);
    }
    mesh_.assign(new_mesh);
}

void MeshViewer::show_notice_loop_subdivision_dialog() {
    if (!show_notice_loop_subdivision) return;
    ImGui::OpenPopup("Notice");
    ImGui::SetNextWindowFocus();
    if (ImGui::BeginPopupModal("Notice", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("Loop subdivision is valid only for triangle mesh");
        if (ImGui::Button("OK")) {
            show_notice_loop_subdivision = false;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

void MeshViewer::show_notice_subdivision_times_dialog() {
    if (!show_notice_subdivision_times) return;
    ImGui::OpenPopup("Notice");
    ImGui::SetNextWindowFocus();
    if (ImGui::BeginPopupModal("Notice", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("Don't let the total subdivision times surpass 5");
        if (ImGui::Button("OK")) {
            show_notice_subdivision_times = false;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}


}