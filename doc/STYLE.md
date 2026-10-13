# Style and abstractions for new code

`multiview` was written by one person over several years, and new code is
expected to read as if the same person wrote it. `doc/coding-style.notes` is
the author's own list; this file adds the conventions that are visible in the
code but not written down there, learned from the 2026 review of the GPU and
build changes. Format with the repo's `.clang-format` (clang-format 14
reproduces the existing files exactly).

## Reuse what already exists

Before adding a helper, look for the one the codebase already has. The three
that were duplicated in 2026 and then unified:

| Need | Use | Not |
|---|---|---|
| Check a CUDA call | `CUDA_CHECK(expr)` in `src/cuda/perceive/graphics/cuda-check.hpp` (throws `std::runtime_error` with the expression and `file:line`) | a private `check_cuda()` per file |
| A device or pinned buffer | `DeviceBuffer` in `src/cuda/perceive/graphics/device-buffer.hpp` (move-only, swap-based assignment, grow-only `create()`) | a nested RAII struct per class |
| A view of an image in device memory | `ImageLayout<T>` in `src/cuda/perceive/graphics/image-layout.hpp` | a new `struct { T* data; unsigned width, height; }` |
| Grid-stride kernel loops | `strided_loop_1d/2d` in `src/cuda/perceive/graphics/strided-loop.hpp` | hand-rolled index math |
| Join strings | `implode(first, last, glue, f)` in `src/perceive/utils/string-utils.hpp` | a stringstream loop |
| Pre/post conditions | `Expects(...)` / `Ensures(...)` | `assert`, early `return` without a log |

## Classes

- Hide CUDA types behind a **Pimpl** (`struct Pimpl; std::unique_ptr<Pimpl> pimpl_;`), as `cuda::slic::SLIC` does. Do not type-erase a `cudaStream_t` as `void*` to keep a header CUDA-free.
- Layout: members first, then construction and assignment, then getters and setters, then actions. Private members and methods end with `_`.
- When CUDA is compiled out, the `#else` stub **throws** `std::runtime_error("CUDA is required")` (see `slic.hpp`); it does not `FATAL`.
- `__device__` code lives in `.hpp` headers (`slic_impl.hpp`), not `.cuh`.

## Functions and files

- A banner above every function: `// ---...--- name`, exactly 80 columns, often followed by a bare `//` line. Kernel files (`*-kernel.cu`, `slic_impl.cu`) are the exception and have none.
- Source files start with a blank line, then `#pragma once` or the first include.
- One declarator per line in structs and for anything but trivial pairs (`int w = 0, h = 0;` is as far as it goes).
- `and` / `or` versus `&&` / `||`: both exist; do not mix them inside one function.
- Member initialisers: `= value` for class members, braces for PODs; do not mix within one header.
- `catch(std::exception& e)` (no `const`), `std::stringstream ss{""}`, `k_` for constants, snake_case everywhere except type names.

## Errors and threads

- Pipeline nodes (`Task::execute`) are `noexcept`. Anything that can throw inside them (CUDA uploads, GPU kernels, file I/O) is wrapped in `try { } catch(std::exception& e) { LOG_ERR(...); }` with a CPU fallback or `return nullptr`. Letting an exception escape terminates the process with no log line.
- Per-thread GPU state (`static thread_local`) multiplies GPU memory by the pool size. Prefer one instance behind a `std::mutex`, as `cuda::Remapper` does, and release it with `register_exit_function`.
- Keep an object alive until every scheduled job that captured `this` has finished: set a flag before `schedule(...)`, clear it as the job's last action, and drain with `try_one_scheduled_job()` in the destructor. Better still, do not schedule work nobody asked for: `SceneDescription` calculates its floor grids on first use.

## Tests

- `CATCH_CONFIG_PREFIX_ALL`, `CATCH_TEST_CASE("Name", "[tag]")`, `CATCH_SECTION`.
- Skip, do not fail, when a GPU or an input file is missing: `WARN("... skipping ..."); return;`. Inputs come from environment variables such as `MULTIVIEW_HYPERPOSE_TEST_IMAGE`.
- Synthetic inputs are preferred (random texture with a known disparity, a known remap) and the assertion compares against the CPU reference with an explicit tolerance.

## Comments

Short, present tense, no narration of what was changed or why a previous
version was wrong. The author's own asides (`// outta here!`) stay.
