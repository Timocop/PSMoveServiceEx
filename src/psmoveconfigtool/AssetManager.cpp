//-- includes -----
#include "AssetManager.h"
#include "Logger.h"

#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#include "stb_image.h"

#include "SDL_error.h"
#include "SDL_opengl.h"

//-- constants -----

// Models and textures
static const char *k_ps3eye_texture_filename = "./assets/models/PS3EyeDiffuse.png";
static const char *k_ps3eye_model_filename = "./assets/models/PS3Eye.obj";

static const char *k_psmove_texture_filename = "./assets/models/PSMoveControllerDiffuse.png";
static const char *k_psmove_model_filename = "./assets/models/PSMoveController.obj";
static const char *k_psmove_bulb_model_filename = "./assets/models/PSMoveControllerBulb.obj";

static const char *k_morpheus_texture_filename = "./assets/models/PSMorpheusDiffuse.png";
static const char *k_morpheus_model_filename = "./assets/models/PSMorpheus.obj";
static const char *k_morpheus_led_model_filename = "./assets/models/PSMorpheusLed.obj";
static const char *k_morpheus_bulb_model_filename = "./assets/models/PSMorpheusBulb.obj";

static const char *k_dualshock_texture_filename = "./assets/models/PSDualShockDiffuse.png";
static const char *k_dualshock_model_filename = "./assets/models/PSDualShock.obj";
static const char *k_dualshock_led_model_filename = "./assets/models/PSDualShockLed.obj";

static const char *k_psnavigation_texture_filename = "./assets/models/PSNavigationDiffuse.png";
static const char *k_psnavigation_model_filename = "./assets/models/PSNavigation.obj";

// Icons
static const char *k_icon_settings_filename = "./assets/icons/setting-line-icon.png";
static const char *k_icon_controller_filename = "./assets/icons/gaming-gamepad-icon.png";
static const char *k_icon_tracker_filename = "./assets/icons/image-icon.png";
static const char *k_icon_hmd_filename = "./assets/icons/vr-goggles-icon.png";

static const char *k_icon_update_filename = "./assets/icons/update-icon.png";
static const char *k_icon_update2_filename = "./assets/icons/update2-icon.png";

static const char *k_icon_warning_filename = "./assets/icons/exclamation-warning-triangle-icon.png";
static const char *k_icon_exclamation_filename = "./assets/icons/exclamation-warning-round-icon.png";
static const char *k_icon_usb_filename = "./assets/icons/usb-icon.png";
static const char *k_icon_bluetooth_filename = "./assets/icons/bluetooth-icon.png";
static const char *k_icon_connect_filename = "./assets/icons/link-line-icon.png";
static const char *k_icon_ban_filename = "./assets/icons/ban-sign-line-icon.png";
static const char *k_icon_shield_filename = "./assets/icons/shield-sedo-line-icon.png";
static const char *k_icon_search_filename = "./assets/icons/search-line-icon.png";
static const char *k_icon_check_filename = "./assets/icons/check-mark-line-icon.png";
static const char *k_icon_close_filename = "./assets/icons/close-line-icon.png";

// Fonts
static const char *k_default_font_filename = "./assets/fonts/OpenSans-Regular.ttf";
static const float k_default_font_pixel_height= 18.f;

static const unsigned int k_font_texture_width = 512;
static const unsigned int k_font_texture_height = 512;

static const size_t k_kilo= 1<<10;
static const size_t k_meg= 1<<20;

//-- statics -----
AssetManager *AssetManager::m_instance= NULL;

//-- public methods -----
AssetManager::AssetManager()
    : m_ps3eye_assets()
	, m_psmove_assets()
	, m_psmove_bulb_assets()
	, m_morpheus_assets()
	, m_morpheus_leds_assets()
	, m_morpheus_bulb_assets()
	, m_dualshock_assets()
	, m_dualshock_led_assets()
	, m_psnavigation_assets()
	, m_defaultFont()
	, m_icon_settings_asset()
	, m_icon_controller_asset()
	, m_icon_tracker_asset()
	, m_icon_hmd_asset()
	, m_icon_update_asset()
	, m_icon_update2_asset()
	, m_icon_warning_asset()
	, m_icon_exclamation_asset()
	, m_icon_usb_asset()
	, m_icon_bluetooth_asset()
	, m_icon_connect_asset()
	, m_icon_ban_asset()
	, m_icon_shield_asset()
	, m_icon_search_asset()
	, m_icon_check_asset()
	, m_icon_close_asset()
{
}

AssetManager::~AssetManager()
{
    assert(m_instance== NULL);
}

bool AssetManager::init()
{
    bool failed= false;

	failed |= !loadFont(k_default_font_filename, k_default_font_pixel_height, &m_defaultFont);

	failed |= !loadOBJ(k_ps3eye_model_filename, m_ps3eye_assets.m_vert, m_ps3eye_assets.m_tex, m_ps3eye_assets.m_norm);
	failed |= !loadTexture(k_ps3eye_texture_filename, &m_ps3eye_assets.m_texture);

	failed |= !loadOBJ(k_psmove_model_filename, m_psmove_assets.m_vert, m_psmove_assets.m_tex, m_psmove_assets.m_norm);
	failed |= !loadOBJ(k_psmove_bulb_model_filename, m_psmove_bulb_assets.m_vert, m_psmove_bulb_assets.m_tex, m_psmove_bulb_assets.m_norm);
	failed |= !loadTexture(k_psmove_texture_filename, &m_psmove_assets.m_texture);

	failed |= !loadOBJ(k_morpheus_model_filename, m_morpheus_assets.m_vert, m_morpheus_assets.m_tex, m_morpheus_assets.m_norm);
	failed |= !loadOBJ(k_morpheus_led_model_filename, m_morpheus_leds_assets.m_vert, m_morpheus_leds_assets.m_tex, m_morpheus_leds_assets.m_norm);
	failed |= !loadOBJ(k_morpheus_bulb_model_filename, m_morpheus_bulb_assets.m_vert, m_morpheus_bulb_assets.m_tex, m_morpheus_bulb_assets.m_norm);
	failed |= !loadTexture(k_morpheus_texture_filename, &m_morpheus_assets.m_texture);

	failed |= !loadOBJ(k_dualshock_model_filename, m_dualshock_assets.m_vert, m_dualshock_assets.m_tex, m_dualshock_assets.m_norm);
	failed |= !loadOBJ(k_dualshock_led_model_filename, m_dualshock_led_assets.m_vert, m_dualshock_led_assets.m_tex, m_dualshock_led_assets.m_norm);
	failed |= !loadTexture(k_dualshock_texture_filename, &m_dualshock_assets.m_texture);

	failed |= !loadOBJ(k_psnavigation_model_filename, m_psnavigation_assets.m_vert, m_psnavigation_assets.m_tex, m_psnavigation_assets.m_norm);
	failed |= !loadTexture(k_psnavigation_texture_filename, &m_psnavigation_assets.m_texture);

	failed |= !loadTexture(k_icon_settings_filename, &m_icon_settings_asset);
	failed |= !loadTexture(k_icon_controller_filename, &m_icon_controller_asset);
	failed |= !loadTexture(k_icon_tracker_filename, &m_icon_tracker_asset);
	failed |= !loadTexture(k_icon_hmd_filename, &m_icon_hmd_asset);

	failed |= !loadTexture(k_icon_update_filename, &m_icon_update_asset);
	failed |= !loadTexture(k_icon_update2_filename, &m_icon_update2_asset);

	failed |= !loadTexture(k_icon_warning_filename, &m_icon_warning_asset);
	failed |= !loadTexture(k_icon_exclamation_filename, &m_icon_exclamation_asset);
	failed |= !loadTexture(k_icon_usb_filename, &m_icon_usb_asset);
	failed |= !loadTexture(k_icon_bluetooth_filename, &m_icon_bluetooth_asset);
	failed |= !loadTexture(k_icon_connect_filename, &m_icon_connect_asset);
	failed |= !loadTexture(k_icon_ban_filename, &m_icon_ban_asset);
	failed |= !loadTexture(k_icon_shield_filename, &m_icon_shield_asset);
	failed |= !loadTexture(k_icon_search_filename, &m_icon_search_asset);
	failed |= !loadTexture(k_icon_check_filename, &m_icon_check_asset);
	failed |= !loadTexture(k_icon_close_filename, &m_icon_close_asset);

    if (!failed)
    {
        // Load IMGUI Fonts
        ImGuiIO& io = ImGui::GetIO();

		io.Fonts->AddFontFromFileTTF(k_default_font_filename, k_default_font_pixel_height);
		io.Fonts->AddFontDefault();
    }

    if (!failed)
    {
        m_instance= this;
    }

    return !failed;
}

void AssetManager::destroy()
{
	m_ps3eye_assets.m_texture.dispose();
	m_psmove_assets.m_texture.dispose();
	m_morpheus_assets.m_texture.dispose();
	m_dualshock_assets.m_texture.dispose();
	m_psnavigation_assets.m_texture.dispose();

    m_defaultFont.dispose();

    m_instance= NULL;
}

//-- private methods -----
bool AssetManager::loadOBJ(
		const std::string& path,
		std::vector<float>& outVertices,
		std::vector<float>& outTexCoords,
		std::vector<float>& outnormals) {
	std::vector<glm::vec3> vertices;
	std::vector<glm::vec2> texture;
	std::vector<glm::vec3> normals;
	std::vector<int> vecNormalIndex;
	std::vector<int> vecFaceIndex;
	std::vector<int> vecTextureIndex;

	// $TODO Culture aware? Maybe switch to invarian culture to parse floating-points correctly?
	std::ifstream in(path, std::ios::in);
	if (!in.is_open()) {
		Log_ERROR("AssetManager::loadOBJ", "Cannot open file (%s)", path.c_str());
		return false;
	}

	std::string line;
	while (std::getline(in, line)) {
		if (line.substr(0, 2) == "v ") {
			std::istringstream l(line.substr(2));
			glm::vec3 vert;
			float x, y, z;
			l >> x; l >> y; l >> z;
			vert = glm::vec3(x, y, z);
			vertices.push_back(vert);
		}
		else if (line.substr(0, 2) == "vt") {
			std::istringstream l(line.substr(3));
			glm::vec2 tex;
			float u, v;
			l >> u; l >> v;
			tex = glm::vec2(u, -v); // Mirror texture. Blender seems to do this?
			texture.push_back(tex);
		}
		else if (line.substr(0, 2) == "vn") {
			std::istringstream l(line.substr(3));
			glm::vec3 norm;
			float x, y, z;
			l >> x; l >> y; l >> z;
			norm = glm::vec3(x, y, z);
			normals.push_back(norm);
		}
		else if (line.substr(0, 2) == "f ") {
			int vertexIndex[3], textureIndex[3], normalIndex[3];
			const char* chh = line.c_str();

			if (sscanf(chh, "f %i/%i/%i %i/%i/%i %i/%i/%i",
					&vertexIndex[0], &textureIndex[0], &normalIndex[0],
					&vertexIndex[1], &textureIndex[1], &normalIndex[1],
					&vertexIndex[2], &textureIndex[2], &normalIndex[2]) == 9) {
				// Vertex, UV,  Normals

				for (int i = 0; i < 3; ++i) {
					vecFaceIndex.push_back(vertexIndex[i] - 1);
					vecTextureIndex.push_back(textureIndex[i] - 1);
					vecNormalIndex.push_back(normalIndex[i] - 1);
				}
			}
			else if (sscanf(chh, "f %i//%i %i//%i %i//%i",
					&vertexIndex[0], &normalIndex[0],
					&vertexIndex[1], &normalIndex[1],
					&vertexIndex[2], &normalIndex[2]) == 6) {
				// Vertex, Normals

				for (int i = 0; i < 3; ++i) {
					vecFaceIndex.push_back(vertexIndex[i] - 1);
					vecNormalIndex.push_back(normalIndex[i] - 1);
				}
			}
			else if (sscanf(chh, "f %i/%i %i/%i %i/%i",
					&vertexIndex[0], &textureIndex[0],
					&vertexIndex[1], &textureIndex[1],
					&vertexIndex[2], &textureIndex[2]) == 6) {
				// Vertex, UV

				for (int i = 0; i < 3; ++i) {
					vecFaceIndex.push_back(vertexIndex[i] - 1);
					vecTextureIndex.push_back(textureIndex[i] - 1);
				}
			}
			else {
				Log_ERROR("AssetManager::loadOBJ", "Unsupported face format in line (%s)", chh);
				continue;
			}
		}
	}

	for (unsigned int i = 0; i < vecFaceIndex.size(); i++) {
		outVertices.push_back(vertices[vecFaceIndex[i]].x);
		outVertices.push_back(vertices[vecFaceIndex[i]].y);
		outVertices.push_back(vertices[vecFaceIndex[i]].z);

		if (!vecTextureIndex.empty()) {
			outTexCoords.push_back(texture[vecTextureIndex[i]].x);
			outTexCoords.push_back(texture[vecTextureIndex[i]].y);
		}

		if (!vecNormalIndex.empty()) {
			outnormals.push_back(normals[vecNormalIndex[i]].x);
			outnormals.push_back(normals[vecNormalIndex[i]].y);
			outnormals.push_back(normals[vecNormalIndex[i]].z);
		}
	}
	return true;
}


bool AssetManager::loadTexture(const char *filename, TextureAsset *textureAsset)
{
    bool success= false;

    int pixelWidth=0, pixelHeight=0, channelCount=0;
	stbi_uc *image_buffer = stbi_load(filename, &pixelWidth, &pixelHeight, &channelCount, 4);

    if (image_buffer != NULL)
    {
		// Determine the OpenGL format based on the number of channels in the image
		GLint glPixelFormat = (channelCount == 4) ? GL_RGBA : GL_RGB;
		GLint internalFormat = (channelCount == 4) ? GL_RGBA : GL_RGB;

		if (channelCount == 3 || channelCount == 4)
		{
			success = textureAsset->init(pixelWidth, pixelHeight, glPixelFormat, internalFormat, image_buffer);
		}
		else
		{
			Log_ERROR("AssetManager::loadTexture", "Image isn't in a supported pixel format (RGB24 or RGBA32)!");
		}
        stbi_image_free(image_buffer);
    }
    else
    {
        Log_ERROR("AssetManager::loadTexture", "Failed to load: %s(%s)", filename, stbi_failure_reason());
    }

    return success;
}

bool AssetManager::loadFont(const char *filename, const float pixelHeight, FontAsset *fontAsset)
{
    unsigned char *temp_ttf_buffer = NULL;

    bool success= true;

    // Scratch buffer for true type font data loaded from file
    temp_ttf_buffer = NULL;

    // Load the True Type Font data into memory
    if (success)
    {
        FILE *fp= fopen(k_default_font_filename, "rb");
        if (fp != NULL)
        {
            // obtain file size
            fseek (fp , 0 , SEEK_END);
            size_t fileSize = ftell (fp);
            rewind (fp);

            if (fileSize > 0 && fileSize < 10*k_meg)
            {
                temp_ttf_buffer= new unsigned char[fileSize];
                size_t bytes_read= fread(temp_ttf_buffer, 1, fileSize, fp);

                if (bytes_read != fileSize)
                {
                    Log_ERROR("AssetManager::loadFont", "Failed to load font (%s): failed to read expected # of bytes.", filename);
                    success= false;
                }
            }
            else
            {
                Log_ERROR("AssetManager::loadFont", "Failed to load font (%s): file size invalid", filename);
                success= false;
            }

            fclose(fp);
        }
        else
        {
            Log_ERROR("AssetManager::loadFont", "Failed to open font file (%s)", filename);
            success= false;
        }
    }

    // Build the sprite sheet for the font
    if (success && !fontAsset->init(temp_ttf_buffer, pixelHeight))
    {
        Log_ERROR("AssetManager::loadFont", "Failed to fit font(%s) into %dx%d sprite texture", 
            filename, k_font_texture_width, k_font_texture_height);
        success = false;
    }
    
    // Free true type font scratch buffers
    if (temp_ttf_buffer != NULL)
    {
        delete[] temp_ttf_buffer;
    }

    return success;
}

//-- Font Asset -----
bool TextureAsset::init(
    unsigned int width,
    unsigned int height,
    unsigned int texture_format,
    unsigned int buffer_format,
    unsigned char *buffer)
{
    bool success = false;

    if (width > 0 && height > 0 && texture_format > 0 && buffer_format > 0)
    {
        this->texture_width = width;
        this->texture_height = height;
        this->texture_format = texture_format;
        this->buffer_format = buffer_format;

        // Setup the OpenGL texture to render the video frame into
        glGenTextures(1, &texture_id);
        glBindTexture(GL_TEXTURE_2D, texture_id);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            texture_format,
            width,
            height,
            0,
            buffer_format,
            GL_UNSIGNED_BYTE,
            buffer);
        glBindTexture(GL_TEXTURE_2D, 0);

        success = true;
    }

    return success;
}

void TextureAsset::copyBufferIntoTexture(const unsigned char *pixels)
{
    if (texture_id != 0)
    {
        glPixelStorei(GL_UNPACK_SWAP_BYTES, GL_FALSE);
        glPixelStorei(GL_UNPACK_LSB_FIRST, GL_TRUE);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
        glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        glBindTexture(GL_TEXTURE_2D, texture_id);
        glTexSubImage2D(
            GL_TEXTURE_2D,
            0,
            0,
            0,
            texture_width,
            texture_height,
            buffer_format,
            GL_UNSIGNED_BYTE,
            pixels);
        glBindTexture(GL_TEXTURE_2D, 0);
    }
}

void TextureAsset::dispose()
{
    // Free the OpenGL video texture
    if (texture_id != 0)
    {
        glDeleteTextures(1, &texture_id);
        texture_id = 0;
        texture_width = 0;
        texture_height = 0;
        texture_format = 0;
        buffer_format = 0;
    }
}

//-- Font Asset -----
bool FontAsset::init(
    unsigned char *ttf_buffer,
    float pixel_height)
{
    bool success = false;

    // Temp buffer to bake the font texture into
    unsigned char *texture_buffer = new unsigned char[k_font_texture_width*k_font_texture_height];

    glyphPixelHeight = pixel_height;

    // Generate the texture for the font sprite sheet
    if (stbtt_BakeFontBitmap(
            ttf_buffer, 0,
            pixel_height,
            texture_buffer, k_font_texture_width, k_font_texture_height,
            32, 96, cdata) > 0)
    {
        // Load the texture into video memory
        success = TextureAsset::init(k_font_texture_width, k_font_texture_height, GL_ALPHA, GL_ALPHA, texture_buffer);
    }

    // Free the font texture buffer
    delete[] texture_buffer;

    return success;
}
