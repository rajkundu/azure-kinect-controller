#include <iostream>
#include <vector>
#include <chrono>
#include <cstdlib>

#include <k4a/k4a.hpp>
#include <k4arecord/record.hpp>

#include "BS_thread_pool.hpp"
#include "SPSCQueue.h"
#include "nfd.hpp"

#include <GLFW/glfw3.h>
#include <GL/glcorearb.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "utils.hpp"

int main(int argc, char* argv[])
{
    int return_code = 0;

    /***************************************
     *          DEAR IMGUI SETUP           *
     ***************************************/

    // Setup window
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()){
        return 1;
    }
    // GL 3.3 + GLSL 330
    const char* glsl_version = "#version 330";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

    // Create window with graphics context
    GLFWwindow* window = glfwCreateWindow(1280, 720, "Azure Kinect DK Controller", NULL, NULL);
    if (window == NULL){
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Set up Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;       // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;           // Enable Docking
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;         // Enable Multi-Viewport / Platform Windows

    // Set up Dear ImGui style
    ImGui::StyleColorsDark();

    // When viewports are enabled we tweak WindowRounding/WindowBg so platform windows can look identical to regular ones.
    ImGuiStyle& style = ImGui::GetStyle();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
        style.WindowRounding = 0.0f;
        style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    /***************************************
     *         AZURE KINECT SETUP          *
     ***************************************/

    std::vector<std::string> available_device_nicknames;
    std::shared_ptr<bool[]> available_device_checkboxes;
    std::shared_ptr<bool[]> available_device_checkboxes_last;
    std::vector<std::string> available_device_serials;
    std::vector<int> device_idxs;
    std::vector<k4a::device> devices;
    std::vector<k4a::record> recordings;
    std::vector<std::string> device_serials;
    std::vector<std::string> device_nicknames;

    // Swizzle masks
    GLint bgra_swizzle_mask[] = {GL_BLUE, GL_GREEN, GL_RED, GL_ALPHA};
    GLint red_as_grayscale_swizzle_mask[] = {GL_RED, GL_RED, GL_RED, GL_ONE};

    // Logging
    std::vector<std::string> k4a_log_msgs;
    if (k4a_set_debug_message_handler(k4a_log_callback, static_cast<void*>(&k4a_log_msgs), K4A_LOG_LEVEL_ERROR) == K4A_RESULT_SUCCEEDED){
        _putenv("K4A_ENABLE_LOG_TO_STDOUT=0");
    }

    /***************************************
     *              MAIN LOOP              *
     ***************************************/
    int num_available_devices = 0;
    int last_num_available_devices = 0;
    int num_enabled_devices = 0;
    bool streaming = false;
    std::shared_ptr<BS::thread_pool> thread_pool;

    bool identical_configs = true;
    bool json_loaded_flag = false;
    std::vector<k4a_device_configuration_t> configs;

    std::vector<std::unique_ptr<rigtorp::SPSCQueue<std::shared_ptr<Image<uint8_t>>>>> color_queues;
    std::vector<std::unique_ptr<rigtorp::SPSCQueue<std::shared_ptr<Image<uint8_t>>>>> ir_queues;
    std::vector<std::shared_ptr<Image<uint8_t>>> color_disps;
    std::vector<std::shared_ptr<Image<uint8_t>>> ir_disps;
    std::vector<ImVec2> color_shapes;
    std::vector<ImVec2> ir_shapes;
    std::vector<GLuint> color_textures;
    std::vector<GLuint> ir_textures;

    bool recording_enabled = false;
    bool continuous_recording = true;
    std::string recording_save_path;
    std::vector<bool> recording_write_enables;
    std::vector<bool> color_hflips;
    std::vector<bool> ir_hflips;

    bool show_debug_window = false;

    try {
        while (!glfwWindowShouldClose(window))
        {
            /*******************
             *  AZURE KINECT   *
             *******************/

            // Only update available devices before streaming
            if (!streaming){
                num_available_devices = k4a::device::get_installed_count();
            }

            if (streaming){
                for (int i = 0; i < num_enabled_devices; i++){

                    // Get capture
                    std::shared_ptr<k4a::capture> capture = std::make_shared<k4a::capture>(k4a::capture());
                    bool success = devices[i].get_capture(capture.get(), std::chrono::milliseconds(5));
                    if (capture->is_valid()){
                        thread_pool->push_task(process_capture, capture, configs[i], color_queues[i].get(), ir_queues[i].get(), color_hflips[i], ir_hflips[i], recording_enabled ? &recordings[i] : nullptr, recording_enabled && (continuous_recording || recording_write_enables[i]));
                        recording_write_enables[i] = false;
                    }

                    if (!color_queues[i]->empty()){
                        color_disps[i] = *(color_queues[i]->front());
                        color_queues[i]->pop();
                    }
                    if (color_disps[i] != nullptr){
                        unsigned int width = color_disps[i]->width();
                        unsigned int height = color_disps[i]->height();

                        // Create GL textures
                        glBindTexture(GL_TEXTURE_2D, color_textures[i]);
                        
                        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                        glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, bgra_swizzle_mask);

                        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
                        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, color_disps[i]->get_buffer());
                        color_shapes[i] = ImVec2(width, height);
                    }

                    if (!ir_queues[i]->empty()){
                        ir_disps[i] = *(ir_queues[i]->front());
                        ir_queues[i]->pop();
                    }
                    if (ir_disps[i] != nullptr){
                        unsigned int width = ir_disps[i]->width();
                        unsigned int height = ir_disps[i]->height();

                        // Create GL textures
                        glBindTexture(GL_TEXTURE_2D, ir_textures[i]);
                        
                        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_G, GL_RED); // use red channel for green
                        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_B, GL_RED); // use red channel for blue

                        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
                        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RED, GL_UNSIGNED_BYTE, ir_disps[i]->get_buffer());
                        ir_shapes[i] = ImVec2(width, height);
                    }
                }
            }

            /*******************
             *       GUI       *
             *******************/

            glfwPollEvents();

            // Start the Dear ImGui frame
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            ImGuiID dockspace_id = ImGui::DockSpaceOverViewport();

            ImGui::SetNextWindowDockID(dockspace_id);
            ImGui::Begin("Control", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
            if (ImGui::BeginMainMenuBar()){
                if (ImGui::BeginMenu("View")){
                    ImGui::MenuItem(show_debug_window ? "Hide Debug Window" : "Show Debug Window", NULL, &show_debug_window);
                    ImGui::EndMenu();
                }
                ImGui::EndMainMenuBar();
            }
            bool enabled_devices_changed = false;
            if (num_available_devices != last_num_available_devices){
                std::cout << "# available devices changed from " << last_num_available_devices << " to " << num_available_devices << std::endl;
                available_device_serials.clear();
                available_device_nicknames.clear();
                available_device_checkboxes = std::shared_ptr<bool[]>(new bool[num_available_devices]);
                available_device_checkboxes_last = std::shared_ptr<bool[]>(new bool[num_available_devices]);
                for (int i = 0; i < num_available_devices; i++){
                    k4a::device device = k4a::device::open(i); 
                    available_device_serials.emplace_back(device.get_serialnum());
                    device.close();

                    available_device_nicknames.emplace_back();
                    available_device_checkboxes[i] = true;
                    available_device_checkboxes_last[i] = true;
                }
                enabled_devices_changed = true;
            }

            for (int i = 0; i < num_available_devices; i++){
                if (available_device_checkboxes[i] != available_device_checkboxes_last[i]){
                    enabled_devices_changed = true;
                    break;
                }
            }
            if (enabled_devices_changed){
                // Reinit device idxs
                device_idxs.clear();
                device_serials.clear();
                for (int i = 0; i < num_available_devices; i++){
                    if (available_device_checkboxes[i]){
                        device_idxs.push_back(i);
                        device_serials.push_back(available_device_serials[i]);
                    }
                }
                num_enabled_devices = device_idxs.size();

                // Config vars
                if (json_loaded_flag){
                    json_loaded_flag = false;
                } else {
                    configs.clear();
                    for(int i = 0; i < num_enabled_devices; i++){
                        configs.push_back(DEFAULT_CONFIG);
                    }
                }
            }

            if (ImGui::CollapsingHeader("Devices", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::BeginDisabled(streaming);
                bool any_devices_selected = false;
                device_nicknames.clear();
                for (int i = 0; i < num_available_devices; i++){
                    available_device_checkboxes_last[i] = available_device_checkboxes[i];
                    ImGui::Checkbox((std::to_string(i) + " -").c_str(), &available_device_checkboxes[i]);
                    ImGui::SameLine();
                    ImGui::PushID(("available_device_nicknames-" + std::to_string(i)).c_str());
                    InputTextWithHintString("", available_device_serials[i].c_str(), &available_device_nicknames[i], ImGuiInputTextFlags_CharsNoBlank);
                    remove_trailing_nulls(available_device_nicknames[i]);
                    if (available_device_checkboxes[i]){
                        if (available_device_nicknames[i].empty()){
                            device_nicknames.push_back(available_device_serials[i]);
                        } else {
                            device_nicknames.push_back(available_device_nicknames[i]);
                        }
                    }
                    ImGui::PopID();
                    any_devices_selected |= available_device_checkboxes[i];
                }

                // Load Config Button
                if (ImGui::Button("Load Config")){
                    NFD::UniquePath json_path;
                    nfdfilteritem_t filter_list[] = {{"JSON File", "json"}};
                    nfdresult_t result = NFD::OpenDialog(json_path, filter_list, 1, nullptr);
                    if (result == NFD_OKAY) {
                        try {
                            load_config_json(
                                json_path.get(),
                                available_device_serials,
                                available_device_nicknames,
                                available_device_checkboxes,
                                &identical_configs,
                                configs,
                                &recording_enabled,
                                &continuous_recording,
                                recording_save_path
                            );
                            json_loaded_flag = true;
                        } catch (std::exception& e){
                            print_error_info(e);
                        }
                    } else if (result != NFD_CANCEL) {
                        printf("Error: %s\n", NFD::GetError() );
                    }
                }

                // Save Config Button
                if (ImGui::Button("Save Config")){
                    NFD::UniquePath json_path;
                    nfdfilteritem_t filter_list[] = {{"JSON File", "json"}};
                    nfdresult_t result = NFD::SaveDialog(json_path, filter_list, 1, nullptr, nullptr);
                    if (result == NFD_OKAY) {
                        std::string json_save_path(json_path.get());
                        save_config_json(
                            json_save_path,
                            identical_configs && num_enabled_devices > 1,
                            available_device_serials,
                            available_device_nicknames,
                            available_device_checkboxes,
                            configs,
                            recording_save_path,
                            continuous_recording
                        );
                    } else if (result != NFD_CANCEL) {
                        printf("Error: %s\n", NFD::GetError() );
                    }
                }

                if (num_enabled_devices == 0){
                    ImGui::Text(("Please " + std::string(num_available_devices == 0 ? "connect" : "enable") + " at least one device before proceeding.").c_str());
                } else {
                    ImGui::Text("");
                }
                ImGui::EndDisabled();
            }

            // Streaming & Recording
            // Streaming
            if (num_enabled_devices > 0){
                if (ImGui::CollapsingHeader("Streaming", ImGuiTreeNodeFlags_DefaultOpen)){
                    ImGui::BeginDisabled(streaming);
                    if (num_enabled_devices > 1){
                        ImGui::Checkbox("Identical Configs", &identical_configs);
                    }
                    if (ImGui::BeginTabBar("Device Config Tabs")){
                        for (int i = 0; i < num_enabled_devices; i++){
                            if (ImGui::BeginTabItem(identical_configs && num_enabled_devices > 1 ? "All Devices" : device_nicknames[i].c_str(), NULL, ImGuiTabItemFlags_NoPushId)){
                                if (!identical_configs || i == 0){
                                    // Note: the reinterpret_casts here are maybe a bit hacky; it assumes the types underlying the config enum variables can safely be interpreted as ints by ImGui
                                    // However, I think this is a pretty safe assumption because everything here is a small and positive integer
                                    static_assert(std::is_convertible<int, std::underlying_type_t<k4a_image_format_t>>::value, "int cannot be converted to k4a_image_format_t's underlying type");
                                    static_assert(std::is_convertible<int, std::underlying_type_t<k4a_color_resolution_t>>::value, "int cannot be converted to k4a_color_resolution_t's resolution underlying type");
                                    static_assert(std::is_convertible<int, std::underlying_type_t<k4a_depth_mode_t>>::value, "int cannot be converted to k4a_depth_mode_t's resolution underlying type");
                                    static_assert(std::is_convertible<int, std::underlying_type_t<k4a_fps_t>>::value, "int cannot be converted to k4a_fps_t's resolution underlying type");

                                    ImGui::PushID(("Combo_Color_Format-" + device_serials[i]).c_str());
                                    ImGui::Combo("", reinterpret_cast<int*>(&configs[i].color_format), COLOR_FORMAT_NAMES.data(), COLOR_FORMAT_NAMES.size());
                                    ImGui::SameLine();
                                    ImGui::Text("Color Format");
                                    ImGui::PopID();

                                    ImGui::PushID(("Combo_Color_Resolution-" + device_serials[i]).c_str());
                                    ImGui::Combo("", reinterpret_cast<int*>(&configs[i].color_resolution), COLOR_RESOLUTION_NAMES.data(), COLOR_RESOLUTION_NAMES.size());
                                    ImGui::SameLine();
                                    ImGui::Text("Color Resolution");
                                    ImGui::PopID();

                                    ImGui::PushID(("Combo_Depth_Mode-" + device_serials[i]).c_str());
                                    ImGui::Combo("", reinterpret_cast<int*>(&configs[i].depth_mode), DEPTH_MODE_NAMES.data(), DEPTH_MODE_NAMES.size());
                                    ImGui::SameLine();
                                    ImGui::Text("Depth Mode");
                                    ImGui::PopID();

                                    ImGui::PushID(("Combo_FPS_Mode-" + device_serials[i]).c_str());
                                    ImGui::Combo("", reinterpret_cast<int*>(&configs[i].camera_fps), FPS_MODE_NAMES.data(), FPS_MODE_NAMES.size());
                                    ImGui::SameLine();
                                    ImGui::Text("FPS");
                                    ImGui::PopID();

                                    ImGui::PushID(("Combo_Sync_mode-" + device_serials[i]).c_str());
                                    ImGui::Combo("", reinterpret_cast<int*>(&configs[i].wired_sync_mode), SYNC_MODE_NAMES.data(), SYNC_MODE_NAMES.size());
                                    ImGui::SameLine();
                                    ImGui::Text("Sync Mode");
                                    ImGui::PopID();

                                    ImGui::EndTabItem();
                                } else {
                                    configs[i] = configs[0];
                                }
                            }
                        }
                        ImGui::EndTabBar();
                    }
                    ImGui::EndDisabled();

                    // Streaming button
                    ImGui::BeginDisabled(num_enabled_devices == 0);
                    if (!streaming){
                        push_button_style_green();
                    } else {
                        push_button_style_amber();
                    }
                    if (ImGui::Button(streaming ? "Stop Streaming" : "Start Streaming")){
                        if (!streaming){
                            try {
                                // First, open devices
                                open_devices(device_idxs, devices);
                                num_enabled_devices = devices.size();

                                // Initialize thread variables
                                initialize_device_thread_vars(num_enabled_devices, thread_pool, color_queues, ir_queues, color_disps, ir_disps, color_shapes, ir_shapes, color_textures, ir_textures, color_hflips, ir_hflips);

                                // Recordings
                                initialize_recordings(recording_enabled, recording_write_enables, recordings, devices, configs, device_idxs, available_device_serials, available_device_nicknames, recording_save_path);

                                // Start streaming
                                start_streaming(devices, configs);
                                streaming = true;
                            } catch (k4a::error& e){
                                print_error_info(e, "Error starting streaming");
                                stop_streaming(devices, configs, recordings);
                                streaming = false;
                            }
                        } else {
                            stop_streaming(devices, configs, recordings);
                            streaming = false;
                        }
                    }
                    pop_button_style();
                    YSpace(10);
                    ImGui::EndDisabled();
                }

                // Recording
                if (ImGui::CollapsingHeader("Recording", ImGuiTreeNodeFlags_DefaultOpen)){
                    ImGui::BeginDisabled(streaming);
                    if (recording_save_path.empty()){
                        ImGui::Text("Recording Disabled (No Save Path Set)");
                        if (ImGui::Button("Set Save Path...")){
                            NFD::UniquePath out_path;
                            nfdresult_t result = NFD::PickFolder(out_path);
                            if (result == NFD_OKAY) {
                                recording_save_path.assign(out_path.get());
                                recording_enabled = true;
                            } else if (result != NFD_CANCEL){
                                printf("Error: %s\n", NFD::GetError() );
                            }
                        }
                    } else {
                        ImGui::Text(("Saving recordings to '" + recording_save_path + "'").c_str());
                        if (ImGui::Button("Cancel")){
                            recording_save_path.clear();
                            recording_enabled = false;
                        }
                        ImGui::Checkbox("Continuous Recording", &continuous_recording);
                    }
                    ImGui::EndDisabled();
                    if (recording_enabled && !continuous_recording && streaming && ImGui::Button("Save Captures")){
                        for (int i = 0; i < num_enabled_devices; i++){
                            recording_write_enables[i] = true;
                        }
                    }
                }
            } else {
                ImGui::BeginDisabled();
                ImGui::PushID("Empty_Collapsing_Headers");
                ImGui::CollapsingHeader("Streaming");
                ImGui::CollapsingHeader("Recording");
                ImGui::PopID();
                ImGui::EndDisabled();
            }

            if (!k4a_log_msgs.empty())
                ImGui::OpenPopup("Error");
            ImVec2 center = ImGui::GetMainViewport()->GetCenter();
            ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
            ImGui::SetNextWindowSize(ImVec2(640, 320), ImGuiCond_Appearing);
            if (ImGui::BeginPopupModal("Error", NULL, ImGuiWindowFlags_Modal | ImGuiWindowFlags_NoSavedSettings))
            {
                ImGui::PushTextWrapPos(ImGui::GetWindowContentRegionWidth());
                ImGui::Text("The following error(s) occurred:");
                ImGui::Separator();
                std::string disp;
                for (int i = 0; i < k4a_log_msgs.size(); i++){
                    if (i > 0){
                        disp.append("\n\n");    
                    }
                    disp.append("[" + std::to_string(i) + "] " + k4a_log_msgs[i]);
                }
                ImGui::Text(disp.c_str());
                ImGui::PopTextWrapPos();
                ImGui::Separator();
                if (ImGui::Button("Close")){
                    ImGui::CloseCurrentPopup();
                    k4a_log_msgs.clear();
                }
                ImGui::EndPopup();
            }

            ImGui::End();

            // Debug window
            if (show_debug_window){
                ImGui::SetNextWindowSize(ImVec2(200, 100), ImGuiCond_Appearing);
                ImGui::Begin("Debug Info");
                ImGui::PushTextWrapPos(ImGui::GetWindowContentRegionWidth());
                ImGui::Text(("Running threads: " + std::to_string(streaming ? thread_pool->get_tasks_running() : 0)).c_str());
                ImGui::Text(("Queued threads: " + std::to_string(streaming ? thread_pool->get_tasks_queued() : 0)).c_str());
                ImGui::Text("Average FPS: %.1f", ImGui::GetIO().Framerate);
                ImGui::PopTextWrapPos();
                ImGui::End();
            }

            // Create window for each camera
            if (streaming){
                ImGui::PushStyleVar(ImGuiStyleVar_WindowMinSize, ImVec2(320, 180));
                for (int i = 0; i < num_enabled_devices; i++){
                    bool show_save_capture_btn = recording_enabled && !continuous_recording;

                    if (color_disps[i] != nullptr){
                        ImGui::Begin((device_nicknames[i] + ": Color").c_str());
                        ImVec2 disp_area = ImGui::GetWindowContentRegionMax();
                        disp_area.y -= ImGui::GetFont()->FontSize + 2 * ImGui::GetStyle().FramePadding.y + (show_save_capture_btn ? 2 * ImGui::GetTextLineHeight() : 0) + 2 * ImGui::GetTextLineHeight();
                        ImGui::Image(reinterpret_cast<void*>(static_cast<intptr_t>(color_textures[i])), get_img_disp_size(color_shapes[i], disp_area));

                        bool color_hflip_temp = color_hflips[i];
                        ImGui::Checkbox("Flip", &color_hflip_temp);
                        color_hflips[i] = color_hflip_temp;

                        if (show_save_capture_btn && ImGui::Button("Save Capture")){
                            recording_write_enables[i] = true;
                        }
                        ImGui::End();
                    }

                    if (ir_disps[i] != nullptr){
                        ImGui::Begin((device_nicknames[i] + ": IR").c_str());
                        ImVec2 disp_area = ImGui::GetWindowContentRegionMax();
                        disp_area.y -= ImGui::GetFont()->FontSize + 2 * ImGui::GetStyle().FramePadding.y + (show_save_capture_btn ? 2 * ImGui::GetTextLineHeight() : 0) + 2 * ImGui::GetTextLineHeight();
                        ImGui::Image(reinterpret_cast<void*>(static_cast<intptr_t>(ir_textures[i])), get_img_disp_size(ir_shapes[i], disp_area));

                        bool ir_hflip_temp = ir_hflips[i];
                        ImGui::Checkbox("Flip", &ir_hflip_temp);
                        ir_hflips[i] = ir_hflip_temp;

                        if (show_save_capture_btn && ImGui::Button("Save Capture")){
                            recording_write_enables[i] = true;
                        }
                        ImGui::End();
                    }
                }
                ImGui::PopStyleVar(1);
            }

            // Rendering
            ImGui::Render();
            int display_w, display_h;
            glfwGetFramebufferSize(window, &display_w, &display_h);
            glViewport(0, 0, display_w, display_h);
            glClear(GL_COLOR_BUFFER_BIT);
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

            // Update and Render additional Platform Windows
            if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
            {
                GLFWwindow* backup_current_context = glfwGetCurrentContext();
                ImGui::UpdatePlatformWindows();
                ImGui::RenderPlatformWindowsDefault();
                glfwMakeContextCurrent(backup_current_context);
            }

            glfwSwapBuffers(window);
            last_num_available_devices = num_available_devices;
        }
    } catch (const std::exception& e) {
        std::cout << "\n\n\n*** UNHANDLED EXCEPTION ***\n\n\n";
        std::cerr << e.what() << std::endl;
        return_code = 1;
    }
    /***************************************
     *               CLEANUP               *
     ***************************************/

    // Azure Kinect
    // devices vector deletes automatically

    // Gui
    gui_cleanup(num_enabled_devices, color_textures, window);

    std::cout << "Successfully completed cleanup." << std::endl;

    return return_code;
}
