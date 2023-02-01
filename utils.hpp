#include <iostream>
#include <string>
#include <array>
#include <vector>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>

#include <k4a/k4a.hpp>
#include <k4arecord/record.hpp>
#include <turbojpeg.h>

#include "BS_thread_pool.hpp"
#include "SPSCQueue.h"
#include "json.hpp"

#include "imgui/imgui.h"

// Max number of images to keep in display queues
#define IMG_QUEUE_SIZE 3
// Streaming start order
static const std::array DEVICE_STREAMING_START_ORDER {
    K4A_WIRED_SYNC_MODE_STANDALONE,
    K4A_WIRED_SYNC_MODE_SUBORDINATE,
    K4A_WIRED_SYNC_MODE_MASTER
};
// Streaming stop order
static const std::array DEVICE_STREAMING_STOP_ORDER {
    K4A_WIRED_SYNC_MODE_MASTER,
    K4A_WIRED_SYNC_MODE_SUBORDINATE,
    K4A_WIRED_SYNC_MODE_STANDALONE
};
// Default streaming config
static const k4a_device_configuration_t DEFAULT_CONFIG = {
    K4A_IMAGE_FORMAT_COLOR_MJPG,
    K4A_COLOR_RESOLUTION_2160P,
    K4A_DEPTH_MODE_NFOV_UNBINNED,
    K4A_FRAMES_PER_SECOND_30,
    false,
    0,
    K4A_WIRED_SYNC_MODE_STANDALONE,
    0,
    false
};

static const std::array COLOR_FORMAT_NAMES {"MJPG", "NV12 (No Visual)", "YUY2 (No Visual)", "BGRA32"};
static const std::array COLOR_RESOLUTION_NAMES {"OFF", "720p", "1080p", "1440p", "1536p", "2160p", "3072p"};
static const std::array DEPTH_MODE_NAMES {"OFF", "NFOV 2x2 Binned", "NFOV Unbinned", "WFOV 2x2 Binned", "WFOV Unbinned", "Passive IR"};
static const std::array FPS_MODE_NAMES {"5", "15", "30"};
static const std::array SYNC_MODE_NAMES {"Standalone", "Master", "Subordinate"};

/***********************************************************
 *                    HELPERS/UTILITIES                    *
 ***********************************************************/

template <class T, std::size_t n> static int index_of(const std::array<T, n> arr, const T& target){
    for (int i = 0; i < arr.size(); i++){
        if (std::string(arr[i]) == target){
            return i;
        }
    }
    return -1;
}

template <typename T> class Image {
    private:
        unsigned int m_width, m_height, m_channels;
        std::shared_ptr<T[]> m_data_ptr;
    public:
        Image(int height, int width, int channels) : m_height(height), m_width(width), m_channels(channels){
            m_data_ptr = std::shared_ptr<T[]>(new T[height * width * channels]);
        }

        // Getters
        unsigned int height(){ return m_height; }
        unsigned int width(){ return m_width; }
        unsigned int channels(){ return m_channels; }
        T* get_buffer(){ return m_data_ptr.get(); }
        size_t size(){
            return m_height * m_width * m_channels;
        }

        T& operator[] (int idx){ return m_data_ptr[idx] };

        ~Image(){
            // data destruction handled by shared ptr
        }
};

static void glfw_error_callback(const int error, const char* description){
    std::cerr << "Glfw Error" << error << ": " << description << std::endl;
}

static void print_error_info(const std::exception& e, const std::string info_msg = ""){
    std::cerr << "[ERROR]:" << (info_msg.empty() ? "" : " " + info_msg) << std::endl;
    std::cerr << "  Type: " << typeid(e).name() << std::endl;
    std::cerr << "  Info: " << e.what() << std::endl;
}

static void remove_trailing_nulls(std::string& s){
    s.erase(std::find(s.begin(), s.end(), '\0'), s.end());
}

static void start_streaming(std::vector<k4a::device>& devices, const std::vector<k4a_device_configuration_t>& configs){
    for (auto wired_sync_mode : DEVICE_STREAMING_START_ORDER){
        for (int i = 0; i < devices.size(); i++){
            if (configs[i].wired_sync_mode == wired_sync_mode){
                devices[i].start_cameras(&configs[i]);
            }
        }
    }
}

static void stop_streaming(
    std::vector<k4a::device>& devices,
    const std::vector<k4a_device_configuration_t>& configs,
    std::vector<k4a::record>& recordings
    ){
    for (auto wired_sync_mode : DEVICE_STREAMING_STOP_ORDER){
        for (int i = 0; i < devices.size(); i++){
            if (configs[i].wired_sync_mode == wired_sync_mode){
                devices[i].stop_cameras();
            }
        }
    }
    // k4a::recording destructor will call flush & close automatically
    recordings.clear();

    // k4a::device destructor calls close
    devices.clear();
}

static void open_devices(std::vector<int>& device_idxs, std::vector<k4a::device>& devices){
    // Create device handles
    for (const int i : device_idxs){
        devices.emplace_back(k4a::device::open(i));
    }

    // Print device info
    std::cout << "\nDevice No.\tSerial No.\n" << std::string(32, '-') << "\n";
    for (int i = 0; i < devices.size(); i++){
        std::cout << device_idxs[i] << "\t\t" << devices[i].get_serialnum() << "\n";
    }
    std::cout << std::flush;
    return;
}

static void gui_cleanup(const int num_enabled_devices, const std::vector<GLuint>& color_textures, GLFWwindow* window){
    if (color_textures.size() > 0){
        glDeleteTextures(num_enabled_devices, color_textures.data());
    }
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
}

static void k4a_log_callback(void* context, k4a_log_level_t level, const char* file, int line, const char* msg){
    auto k4a_log_msgs = reinterpret_cast<std::vector<std::string>*>(context);
    k4a_log_msgs->push_back(msg);
}

static void load_config_json(
    const std::string& input_file_path,
    std::vector<std::string>& available_device_serials,
    std::vector<std::string>& available_device_nicknames,
    const std::shared_ptr<bool[]> available_device_checkboxes,
    bool* identical_configs,
    std::vector<k4a_device_configuration_t>& configs,
    bool* recording_enabled,
    bool* continuous_recording,
    std::string& recording_save_path
){
    std::ifstream ifs(input_file_path);
    std::string json_str((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));
    json::JSON config_json = json::JSON::Load(json_str);
    if (config_json.IsNull()){
        throw std::runtime_error("Input file '" + input_file_path + "' is not a valid JSON.");
    }

    *identical_configs = config_json["identical_configs"].ToBool();
    if (config_json.hasKey("save_path")){
        std::string save_path = config_json["save_path"].ToStringNoEscape();
        recording_save_path = save_path;
        *recording_enabled = !recording_save_path.empty();
    }
    if (config_json.hasKey("continuous_recording")){
        *continuous_recording = config_json["continuous_recording"].ToBool();
    }

    configs.clear();
    int num_available_devices = available_device_serials.size();
    for (int i = 0; i < num_available_devices; i++){
        std::string serial = available_device_serials[i];
        if (config_json.hasKey(serial)){
            available_device_nicknames[i] = config_json[serial]["nickname"].ToString();
            available_device_checkboxes[i] = true;
            std::string key = *identical_configs ? "*" : serial;

            k4a_device_configuration_t config = DEFAULT_CONFIG;
            config.color_format = static_cast<k4a_image_format_t>(index_of(COLOR_FORMAT_NAMES, config_json[key]["color_format"].ToString().c_str()));
            config.color_resolution = static_cast<k4a_color_resolution_t>(index_of(COLOR_RESOLUTION_NAMES, config_json[key]["color_resolution"].ToString().c_str()));
            config.depth_mode = static_cast<k4a_depth_mode_t>(index_of(DEPTH_MODE_NAMES, config_json[key]["depth_mode"].ToString().c_str()));
            config.camera_fps = static_cast<k4a_fps_t>(config_json[key]["fps"].ToInt());
            config.wired_sync_mode = static_cast<k4a_wired_sync_mode_t>(index_of(SYNC_MODE_NAMES, config_json[key]["sync_mode"].ToString().c_str()));
            configs.push_back(config);
        } else {
            available_device_checkboxes[i] = false;
        }
    }
}

static void save_config_json(
    const std::string& output_file_path,
    const bool identical_configs,
    const std::vector<std::string>& available_device_serials,
    const std::vector<std::string>& available_device_nicknames,
    const std::shared_ptr<bool[]> available_device_checkboxes,
    const std::vector<k4a_device_configuration_t>& configs,
    const std::string& recording_save_path,
    const bool continuous_recording
){
    json::JSON j;
    j["identical_configs"] = identical_configs;
    if (!recording_save_path.empty()){
        j["save_path"] = recording_save_path;
        j["continuous_recording"] = continuous_recording;
    }
    if (identical_configs){
        j["*"]["color_format"] = COLOR_FORMAT_NAMES[configs[0].color_format];
        j["*"]["color_resolution"] = COLOR_RESOLUTION_NAMES[configs[0].color_resolution];
        j["*"]["depth_mode"] = DEPTH_MODE_NAMES[configs[0].depth_mode];
        j["*"]["fps"] = static_cast<int>(configs[0].camera_fps); // stored in JSON as an integer, not string
        j["*"]["sync_mode"] = SYNC_MODE_NAMES[configs[0].wired_sync_mode];
    }
    int opened_device_idx = 0;
    for (int i = 0; i < available_device_serials.size(); i++){
        // Only save opened devices
        if (!available_device_checkboxes[i]){
            continue;
        }
        auto serial = available_device_serials[i];
        j[serial]["nickname"] = available_device_nicknames[i];
        if (!identical_configs){
            if (!j.hasKey(serial)) j[serial] = json::Object();
            j[serial]["color_format"] = COLOR_FORMAT_NAMES[configs[opened_device_idx].color_format];
            j[serial]["color_resolution"] = COLOR_RESOLUTION_NAMES[configs[opened_device_idx].color_resolution];
            j[serial]["depth_mode"] = DEPTH_MODE_NAMES[configs[opened_device_idx].depth_mode];
            j[serial]["fps"] = static_cast<int>(configs[opened_device_idx].camera_fps);  // stored in JSON as an integer, not string
            j[serial]["sync_mode"] = SYNC_MODE_NAMES[configs[opened_device_idx].wired_sync_mode];
        }
        opened_device_idx++;
    }
    std::ofstream(output_file_path) << j.dump() << std::endl;
}

static void initialize_device_thread_vars(
    int num_enabled_devices,
    std::shared_ptr<BS::thread_pool>& thread_pool,
    std::vector<std::unique_ptr<rigtorp::SPSCQueue<std::shared_ptr<Image<uint8_t>>>>>& color_queues,
    std::vector<std::unique_ptr<rigtorp::SPSCQueue<std::shared_ptr<Image<uint8_t>>>>>& ir_queues,
    std::vector<std::shared_ptr<Image<uint8_t>>>& color_disps,
    std::vector<std::shared_ptr<Image<uint8_t>>>& ir_disps,
    std::vector<ImVec2>& color_shapes,
    std::vector<ImVec2>& ir_shapes,
    std::vector<GLuint>& color_textures,
    std::vector<GLuint>& ir_textures,
    std::vector<bool>& color_hflips,
    std::vector<bool>& ir_hflips
){
    // Create threads
    int num_threads = std::min<int>(2 * num_enabled_devices, std::thread::hardware_concurrency() - 1);
    // int num_threads = std::thread::hardware_concurrency() - 1;
    thread_pool = std::shared_ptr<BS::thread_pool>(new BS::thread_pool(num_threads));

    // Image queues for display
    color_queues.clear();
    ir_queues.clear();

    // Display image pointers
    color_disps.clear();
    ir_disps.clear();

    // ImVec2s for ImGui/GL texture generation
    color_shapes.clear();
    ir_shapes.clear();

    // GLuints storing OpenGL textures
    color_textures.clear();
    ir_textures.clear();

    // Booleans
    color_hflips.clear();
    ir_hflips.clear();

    for (int i = 0; i < num_enabled_devices; i++){
        // Create display image queues
        color_queues.push_back(std::move(std::make_unique<rigtorp::SPSCQueue<std::shared_ptr<Image<uint8_t>>>>(IMG_QUEUE_SIZE)));
        ir_queues.push_back(std::move(std::make_unique<rigtorp::SPSCQueue<std::shared_ptr<Image<uint8_t>>>>(IMG_QUEUE_SIZE)));

        // Create display image pointers
        ir_disps.emplace_back();
        color_disps.emplace_back();

        // Create default ImVec2
        color_shapes.emplace_back();
        ir_shapes.emplace_back();

        // Create display OpenGL textures (initialize to 0 = nullptr)
        color_textures.push_back(0);
        ir_textures.push_back(0);

        color_hflips.push_back(false);
        ir_hflips.push_back(false);
    }

    // Generate color/ir textures for display images
    glGenTextures(num_enabled_devices, color_textures.data());
    glGenTextures(num_enabled_devices, ir_textures.data());
}

static void initialize_recordings(
    const bool recording_enabled,
    std::vector<bool>& recording_write_enables,
    std::vector<k4a::record>& recordings,
    const std::vector<k4a::device>& devices,
    const std::vector<k4a_device_configuration_t>& configs,
    const std::vector<int>& device_idxs,
    const std::vector<std::string>& available_device_serials,
    const std::vector<std::string>& available_device_nicknames,
    const std::string& recording_save_path = ""
){
    recording_write_enables.clear();
    recordings.clear();
    if (!recording_enabled){
        for (int i = 0; i < devices.size(); i++){
            recording_write_enables.push_back(false);
        }
        return;
    }

    std::chrono::seconds rec_start_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch());
    for (int i = 0; i < devices.size(); i++){
        recording_write_enables.push_back(false);

        std::string nickname = available_device_nicknames[device_idxs[i]];
        if (nickname.empty()){
            nickname = available_device_serials[device_idxs[i]];
        }
        std::filesystem::path full_path = std::filesystem::path(recording_save_path) / (std::to_string(rec_start_time.count()) + "_" + nickname + ".mkv");
        recordings.emplace_back(k4a::record::create(full_path.string().c_str(), devices[i], configs[i]));
        recordings[i].write_header();
    }
}

void process_capture(
    const std::shared_ptr<k4a::capture> capture,
    const k4a_device_configuration_t& config,
    rigtorp::SPSCQueue<std::shared_ptr<Image<uint8_t>>>* color_queue,
    rigtorp::SPSCQueue<std::shared_ptr<Image<uint8_t>>>* ir_queue,
    const bool hflip_color,
    const bool hflip_ir,
    k4a::record* recording,
    const bool recording_write_enable
){
    // Get image
    k4a::image color_img = capture->get_color_image();
    if (color_img.is_valid()){
        bool success = false;
        unsigned int width = color_img.get_width_pixels();
        unsigned int height = color_img.get_height_pixels();
        std::shared_ptr<Image<uint8_t>> color_disp = std::make_shared<Image<uint8_t>>(height, width, 4);

        // MJPG
        if (color_img.get_format() == K4A_IMAGE_FORMAT_COLOR_MJPG){
            tjhandle jpeg_decompressor = tjInitDecompress();
            int result = tjDecompress2(jpeg_decompressor, color_img.get_buffer(), color_img.get_size(), color_disp->get_buffer(), width, 0/*pitch*/, height, TJPF_BGRA, TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE);
            if (result != 0){
                std::cerr << "[ERROR] Failed to properly decode image\n";
                fprintf(stderr, "Error code:\t%d\n", result);
                fprintf(stderr, "Error str:\t%s\n", tjGetErrorStr2(jpeg_decompressor));
                fprintf(stderr, "Capture:\t%p\n", capture.get());
                fprintf(stderr, "Col img:\t%p\n", color_img.handle());
                fprintf(stderr, "Col img vld:\t%d\n", color_img.is_valid());
                std::cerr << std::flush;
            }
            tjDestroy(jpeg_decompressor);
            success = (result == 0);
        } else if (color_img.get_format() == K4A_IMAGE_FORMAT_COLOR_BGRA32) {
            memcpy(color_disp->get_buffer(), color_img.get_buffer(), color_img.get_size());
            success = true;
        } else {
            // NV12, YUY2 visualization not yet implemented
        }

        if (hflip_color){
            uint32_t* buffer = reinterpret_cast<uint32_t*>(color_disp->get_buffer());
            for (unsigned int v = 0; v < height; v++){
                for (unsigned int u = 0; u < width / 2; u++){
                    unsigned int idx = v * width + u;
                    unsigned int flip_idx = v * width + ((width - 1) - u);
                    uint32_t temp = buffer[idx];
                    buffer[idx] = buffer[flip_idx];
                    buffer[flip_idx] = temp;
                }
            }
        }

        // Add to display color queue
        if (success){
            success &= color_queue->try_push(color_disp);
        }
    }

    k4a::image ir_img = capture->get_ir_image();
    if (ir_img.is_valid()){
        unsigned int width = ir_img.get_width_pixels();
        unsigned int height = ir_img.get_height_pixels();
        std::shared_ptr<Image<uint8_t>> ir_disp = std::make_shared<Image<uint8_t>>(height, width, 1);

        double expected_pixel_range_max = config.depth_mode == K4A_DEPTH_MODE_PASSIVE_IR ? 100.0 : 1000.0; // hardcoded values are from k4aviewer/k4astaticimageproperties.h
        double scale_factor = std::numeric_limits<uint8_t>::max() / expected_pixel_range_max;

        uint16_t* in_buffer = reinterpret_cast<uint16_t*>(ir_img.get_buffer());
        uint8_t* out_buffer = ir_disp->get_buffer();
        for (unsigned int v = 0; v < height; v++){
            for (unsigned int u = 0; u < width; u++){
                unsigned int idx = v * width + u;
                unsigned int flip_idx = v * width + ((width - 1) - u);
                uint16_t scaled_value = in_buffer[idx] * scale_factor;
                uint8_t out_value = scaled_value > std::numeric_limits<uint8_t>::max() ? std::numeric_limits<uint8_t>::max() : scaled_value;
                out_buffer[hflip_ir ? flip_idx : idx] = out_value;
            }
        }

        // Add to display ir queue
        bool success = ir_queue->try_push(ir_disp);
    }

    // Add capture to recording
    if (recording != nullptr && recording_write_enable){
        recording->write_capture(*capture);
    }
    return;
}

/***********************************************************
 *                  GUI HELPERS/UTILITIES                  *
 ***********************************************************/

static void push_style_regular(const ImGuiCol col){
    ImGui::PushStyleColor(col, ImGui::GetStyleColorVec4(col));
}
template <std::size_t N> static void push_style_regular(const ImGuiCol(&cols)[N]){
    for (ImGuiCol col : cols){
        push_style_regular(col);
    }
}

static void push_button_style_red(){
    ImGui::PushStyleColor(ImGuiCol_Button,          IM_COL32(175, 0, 0, 255));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered,   IM_COL32(200, 0, 0, 255));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,    IM_COL32(150, 0, 0, 255));
}
static void push_button_style_green(){
    ImGui::PushStyleColor(ImGuiCol_Button,          IM_COL32(0, 175, 0, 255));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered,   IM_COL32(0, 200, 0, 255));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,    IM_COL32(0, 150, 0, 255));
}
static void push_button_style_amber(){
    ImGui::PushStyleColor(ImGuiCol_Button,          IM_COL32(175, 140, 0, 255));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered,   IM_COL32(200, 160, 0, 255));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,    IM_COL32(150, 120, 0, 255));
}
static void push_button_style_regular(){
    push_style_regular({ImGuiCol_Button, ImGuiCol_ButtonActive, ImGuiCol_ButtonActive});
}
static void pop_button_style(){
    ImGui::PopStyleColor(3);
}

static void XSpace(unsigned int x){
    ImGui::SameLine();
    ImGui::Dummy(ImVec2{static_cast<float>(x), 0.0f});
}
static void YSpace(unsigned int y){
    ImGui::Dummy(ImVec2{0.0f, static_cast<float>(y)});
}

static int InputTextStringResizeCallback(ImGuiInputTextCallbackData* data){
    if (data->EventFlag == ImGuiInputTextFlags_CallbackResize)
    {
        std::string* my_str = static_cast<std::string*>(data->UserData);
        my_str->resize(data->BufSize);
        data->Buf = my_str->data();
    }
    return 0;
}

static bool InputTextString(const char* label, std::string* my_str, ImGuiInputTextFlags flags = 0){
    IM_ASSERT((flags & ImGuiInputTextFlags_CallbackResize) == 0);
    return ImGui::InputText(label, my_str->data(), my_str->size() + 1, flags | ImGuiInputTextFlags_CallbackResize, InputTextStringResizeCallback, static_cast<void*>(my_str));
}

static bool InputTextWithHintString(const char* label, const char* hint, std::string* my_str, ImGuiInputTextFlags flags = 0){
    IM_ASSERT((flags & ImGuiInputTextFlags_CallbackResize) == 0);
    return ImGui::InputTextWithHint(label, hint, my_str->data(), my_str->size() + 1, flags | ImGuiInputTextFlags_CallbackResize, InputTextStringResizeCallback, static_cast<void*>(my_str));
}

// taken from Azure Kinect Viewer GetMaxImageSize
inline ImVec2 get_img_disp_size(const ImVec2& img_size, const ImVec2& img_max_size){
    const float source_aspect = img_size.x / img_size.y;
    const float py = ImGui::GetStyle().WindowPadding.y * 2;
    const float px = ImGui::GetStyle().WindowPadding.x * 2;
    const float max_width = img_max_size.x - px;
    const float max_height = img_max_size.y - py;
    ImVec2 disp_size;
    if (max_width / source_aspect <= max_height)
    {
        disp_size.x = max_width;
        disp_size.y = max_width / source_aspect;
    } else {
        disp_size.x = max_height * source_aspect;
        disp_size.y = max_height;
    }
    return disp_size;
}
