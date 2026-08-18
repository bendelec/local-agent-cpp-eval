#include "lab_state.hpp"

#include <SDL3/SDL.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>

namespace {

struct Camera {
    vwmini::Vec2 center{4.0f, 4.0f};
    float pixels_per_metre{75.0f};
};

[[nodiscard]] SDL_FPoint to_screen(vwmini::Vec2 point, const Camera& camera, int width,
                                   int height) noexcept
{
    return {static_cast<float>(width) * 0.5f + (point.x - camera.center.x) * camera.pixels_per_metre,
            static_cast<float>(height) * 0.5f - (point.y - camera.center.y) * camera.pixels_per_metre};
}

[[nodiscard]] vwmini::Vec2 to_world(float x, float y, const Camera& camera, int width,
                                    int height) noexcept
{
    return {(x - static_cast<float>(width) * 0.5f) / camera.pixels_per_metre + camera.center.x,
            (static_cast<float>(height) * 0.5f - y) / camera.pixels_per_metre + camera.center.y};
}

void set_colour(SDL_Renderer* renderer, SDL_Color colour)
{
    SDL_SetRenderDrawColor(renderer, colour.r, colour.g, colour.b, colour.a);
}

void draw_circle(SDL_Renderer* renderer, SDL_FPoint center, float radius, SDL_Color colour)
{
    set_colour(renderer, colour);
    constexpr int segments = 20;
    SDL_FPoint previous{center.x + radius, center.y};
    for (int index = 1; index <= segments; ++index) {
        const float angle = static_cast<float>(index) / static_cast<float>(segments) * 6.2831853f;
        const SDL_FPoint next{center.x + std::cos(angle) * radius,
                              center.y + std::sin(angle) * radius};
        SDL_RenderLine(renderer, previous.x, previous.y, next.x, next.y);
        previous = next;
    }
}

void draw_cross(SDL_Renderer* renderer, SDL_FPoint center, float radius, SDL_Color colour)
{
    set_colour(renderer, colour);
    SDL_RenderLine(renderer, center.x - radius, center.y - radius, center.x + radius, center.y + radius);
    SDL_RenderLine(renderer, center.x - radius, center.y + radius, center.x + radius, center.y - radius);
}

[[nodiscard]] SDL_Color status_colour(vwmini::AgentStatus status)
{
    switch (status) {
    case vwmini::AgentStatus::Idle: return {170, 170, 170, 255};
    case vwmini::AgentStatus::Moving: return {80, 210, 255, 255};
    case vwmini::AgentStatus::Reached: return {100, 230, 120, 255};
    case vwmini::AgentStatus::NoPath: return {255, 90, 90, 255};
    }
    return {255, 255, 255, 255};
}

[[nodiscard]] const char* status_name(vwmini::AgentStatus status) noexcept
{
    switch (status) {
    case vwmini::AgentStatus::Idle: return "idle";
    case vwmini::AgentStatus::Moving: return "moving";
    case vwmini::AgentStatus::Reached: return "reached";
    case vwmini::AgentStatus::NoPath: return "no path";
    }
    return "unknown";
}

void draw_scene(SDL_Renderer* renderer, const vwmini_lab::LabState& state, const Camera& camera,
                int width, int height, std::optional<vwmini::AgentId> selected,
                std::optional<vwmini::Vec2> spawn, std::optional<vwmini::Vec2> target,
                bool paused)
{
    set_colour(renderer, {20, 24, 30, 255});
    SDL_RenderClear(renderer);

    for (const vwmini::Polygon& triangle : state.triangles()) {
        set_colour(renderer, {100, 150, 185, 255});
        for (std::size_t edge = 0; edge < 3; ++edge) {
            const SDL_FPoint a = to_screen(triangle.vertices[edge], camera, width, height);
            const SDL_FPoint b = to_screen(triangle.vertices[(edge + 1u) % 3u], camera, width, height);
            SDL_RenderLine(renderer, a.x, a.y, b.x, b.y);
        }
        for (const vwmini::Vec2 vertex : triangle.vertices) {
            draw_circle(renderer, to_screen(vertex, camera, width, height), 4.0f, {245, 205, 90, 255});
        }
    }

    if (spawn) {
        draw_circle(renderer, to_screen(*spawn, camera, width, height), 8.0f, {255, 220, 80, 255});
    }
    if (target) {
        draw_cross(renderer, to_screen(*target, camera, width, height), 9.0f, {255, 145, 80, 255});
    }

    const std::vector<vwmini_lab::DisplayAgent> agents = state.agents();
    // Green route guides are independent public-API path queries. The white vector
    // at each agent is its instantaneous crowd-steering velocity, not its nav path.
    for (const vwmini_lab::DisplayAgent& agent : agents) {
        const auto preview = state.route_preview(agent.id);
        if (!preview || preview->points.size() < 2u) {
            continue;
        }
        set_colour(renderer, {85, 185, 110, 255});
        for (std::size_t index = 0; index + 1u < preview->points.size(); ++index) {
            const SDL_FPoint a = to_screen(preview->points[index], camera, width, height);
            const SDL_FPoint b = to_screen(preview->points[index + 1u], camera, width, height);
            SDL_RenderLine(renderer, a.x, a.y, b.x, b.y);
        }
    }

    for (const vwmini_lab::DisplayAgent& agent : agents) {
        const SDL_FPoint point = to_screen(agent.state.position, camera, width, height);
        const float radius = agent.state.radius * camera.pixels_per_metre;
        draw_circle(renderer, point, std::max(radius, 3.0f), status_colour(agent.state.status));
        const SDL_FPoint velocity = to_screen(agent.state.position + agent.state.velocity * 0.35f,
                                              camera, width, height);
        set_colour(renderer, {220, 220, 220, 255});
        SDL_RenderLine(renderer, point.x, point.y, velocity.x, velocity.y);
        if (agent.state.goal) {
            draw_cross(renderer, to_screen(*agent.state.goal, camera, width, height), 5.0f,
                       {130, 210, 130, 255});
        }
        if (selected && *selected == agent.id) {
            draw_circle(renderer, point, std::max(radius, 3.0f) + 4.0f, {255, 255, 255, 255});
        }
        set_colour(renderer, status_colour(agent.state.status));
        SDL_RenderDebugTextFormat(renderer, point.x + radius + 5.0f, point.y - 5.0f,
                                  "#%u %s v=%.2f", agent.id.value,
                                  status_name(agent.state.status), length(agent.state.velocity));
    }

    set_colour(renderer, {235, 235, 235, 255});
    SDL_RenderDebugText(renderer, 12.0f, 12.0f,
                        "Left: spawn/select  Right: target  A: add  G: selected goal");
    SDL_RenderDebugText(renderer, 12.0f, 26.0f,
                        "Green: nav route  White: steering velocity  Space: pause  N: step  R: reset");
    SDL_RenderDebugTextFormat(renderer, 12.0f, 40.0f, "%s | agents: %zu | %s",
                              paused ? "PAUSED" : "RUNNING", agents.size(),
                              state.diagnostic().c_str());

    SDL_RenderPresent(renderer);
}

} // namespace

int main()
{
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL initialization failed: %s", SDL_GetError());
        return 1;
    }
    SDL_Window* window = SDL_CreateWindow("VWmini lab", 1100, 800, SDL_WINDOW_RESIZABLE);
    SDL_Renderer* renderer = window ? SDL_CreateRenderer(window, nullptr) : nullptr;
    if (!renderer) {
        SDL_Log("SDL renderer initialization failed: %s", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    vwmini_lab::LabState state;
    Camera camera;
    std::optional<vwmini::Vec2> spawn;
    std::optional<vwmini::Vec2> target;
    std::optional<vwmini::AgentId> selected;
    std::optional<std::pair<std::size_t, std::size_t>> dragged_vertex;
    bool panning = false;
    bool paused = false;
    std::uint64_t previous_ticks = SDL_GetTicks();
    bool running = true;

    while (running) {
        int width{};
        int height{};
        SDL_GetWindowSizeInPixels(window, &width, &height);
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT) {
                running = false;
            } else if (event.type == SDL_EVENT_MOUSE_WHEEL) {
                camera.pixels_per_metre = std::clamp(camera.pixels_per_metre *
                                                          (event.wheel.y > 0.0f ? 1.15f : 0.87f),
                                                      20.0f, 300.0f);
            } else if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
                const vwmini::Vec2 world = to_world(event.button.x, event.button.y, camera, width, height);
                if (event.button.button == SDL_BUTTON_MIDDLE) {
                    panning = true;
                } else if (event.button.button == SDL_BUTTON_RIGHT) {
                    target = world;
                } else if (event.button.button == SDL_BUTTON_LEFT) {
                    selected = state.nearest_agent(world, 0.35f);
                    if (!selected) {
                        spawn = world;
                        float nearest = 12.0f;
                        for (std::size_t triangle = 0; triangle < state.triangles().size(); ++triangle) {
                            for (std::size_t vertex = 0; vertex < 3; ++vertex) {
                                const SDL_FPoint screen = to_screen(
                                    state.triangles()[triangle].vertices[vertex], camera, width, height);
                                const float dx = screen.x - event.button.x;
                                const float dy = screen.y - event.button.y;
                                const float distance = std::sqrt(dx * dx + dy * dy);
                                if (distance < nearest) {
                                    nearest = distance;
                                    dragged_vertex = {triangle, vertex};
                                }
                            }
                        }
                    }
                }
            } else if (event.type == SDL_EVENT_MOUSE_BUTTON_UP) {
                if (event.button.button == SDL_BUTTON_MIDDLE) {
                    panning = false;
                }
                if (event.button.button == SDL_BUTTON_LEFT && dragged_vertex) {
                    if (state.rebuild_mesh()) {
                        selected.reset();
                    }
                    dragged_vertex.reset();
                }
            } else if (event.type == SDL_EVENT_MOUSE_MOTION) {
                if (panning) {
                    camera.center.x -= event.motion.xrel / camera.pixels_per_metre;
                    camera.center.y += event.motion.yrel / camera.pixels_per_metre;
                }
                if (dragged_vertex) {
                    const auto [triangle, vertex] = *dragged_vertex;
                    state.triangles()[triangle].vertices[vertex] =
                        to_world(event.motion.x, event.motion.y, camera, width, height);
                }
            } else if (event.type == SDL_EVENT_KEY_DOWN && !event.key.repeat) {
                switch (event.key.key) {
                case SDLK_SPACE: paused = !paused; break;
                case SDLK_N: state.advance(1.0f / 60.0f); break;
                case SDLK_R: state.reset_default_mesh(); selected.reset(); break;
                case SDLK_A:
                    if (spawn && target) static_cast<void>(state.add_agent(*spawn, *target));
                    break;
                case SDLK_G:
                    if (selected && target) static_cast<void>(state.set_goal(*selected, *target));
                    break;
                case SDLK_DELETE:
                    if (selected) static_cast<void>(state.remove_agent(*selected));
                    selected.reset();
                    break;
                default: break;
                }
            }
        }

        const std::uint64_t now = SDL_GetTicks();
        const float elapsed = std::min(static_cast<float>(now - previous_ticks) / 1000.0f, 0.1f);
        previous_ticks = now;
        if (!paused) {
            state.advance(elapsed);
        }
        const std::string title = std::string{"VWmini lab | "} + state.diagnostic() +
                                  " | L spawn/select, R target, A add, G goal, Space pause, N step";
        SDL_SetWindowTitle(window, title.c_str());
        draw_scene(renderer, state, camera, width, height, selected, spawn, target, paused);
        SDL_Delay(1);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
