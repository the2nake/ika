#include <cmath>
#include <fstream>
#include <functional>
#include <numbers>
#include <print>
#include <random>

#include "KinematicIntegrator.hpp"
#include "KinematicIntegrator.ipp"

using namespace std;

struct Vec2 {
  Vec2() : mod(0.0), arg(0.0) {}
  Vec2(double mod, double arg) : mod(mod), arg(arg) {}

  double mod;
  double arg;

  double norm() { return mod * mod + arg * arg; }

  void operator+=(const Vec2& b) {
    mod += b.mod;
    arg += b.arg;
  }

  void operator-=(const Vec2& b) {
    mod -= b.mod;
    arg -= b.arg;
  }
};

bool operator==(const Vec2& a, const Vec2& b) {
  return a.mod == b.mod && a.arg == b.arg;
}
Vec2 operator*(double x, const Vec2& a) { return {x * a.mod, x * a.arg}; }
Vec2 operator*(const Vec2& a, double x) { return operator*(x, a); }
Vec2 operator/(const Vec2& a, double x) { return (1 / x) * a; }
Vec2 operator+(const Vec2& a, const Vec2& b) {
  return {a.mod + b.mod, a.arg + b.arg};
}
Vec2 operator-(const Vec2& a, const Vec2& b) { return a + (-1.0 * b); }

template <NormedVector V, NormedVector V2>
class Eval {
 public:
  Eval(V target, std::function<V(V2)> gen) : target(target), gen(gen) {}
  double operator()(V2 v) const {
    V output = gen(v);
    double norm = (target - output).norm();
    return norm * norm;
  };
  const V target;
  const std::function<V(V2)> gen;
};

// sling issue where points diverge away from the center of mass

int main() {
  const double runtime = 0.5, prec = 1e5;
  const int samples = 9;

  Vec2 target(3, 3);
  const double E_mod = 3 * sqrt(2), V_mod = 0.1 * E_mod;
  const double E_arg = numbers::pi / 4, V_arg = 0.1 * E_arg;

  auto generator = [](const Vec2& v2) {
    return Vec2{v2.mod * std::cos(v2.arg), v2.mod * std::sin(v2.arg)};
  };
  Eval<Vec2, Vec2> eval(target, generator);

  // use a seed sequence to prevent under-seeding mt19937
  std::random_device r;
  std::seed_seq ss{r(), r(), r(), r(), r(), r(), r(), r(), r()};
  std::mt19937 e(std::random_device{}());
  std::uniform_real_distribution<> dist_mod(E_mod - V_mod, E_mod + V_mod);
  std::uniform_real_distribution<> dist_arg(E_arg - V_arg, E_arg + V_arg);

  std::vector<Vec2> guesses;
  for (int i = 0; i < samples; ++i) {
    guesses.emplace_back(dist_mod(e), dist_arg(e));
  }
  // const int steps = sqrt(samples);
  // for (double i = 0.0; i <= 1.01; i += 1.0 / steps) {
  //   for (double j = 0.0; j <= 1.01; j += 1.0 / steps) {
  //     guesses.emplace_back(lerp(E_mod - V_mod, E_mod + V_mod, i),
  //                          lerp(E_arg - V_arg, E_arg + V_arg, j));
  //   }
  // }
  GSA<Vec2, Eval<Vec2, Vec2>> integrator(guesses, eval, 1 / prec);

  std::vector<Vec2> trace;
  string filename = "out/trace.txt";
  std::ofstream out_file(filename);
  if (!out_file.is_open()) {
    println("{} could not be opened", filename);
    return 1;
  }

  // reduce the gravitational constant over time

  int best_iter = 0;
  double best_score = 1000.0;
  Vec2 fit;
  for (int i = 0; i < runtime * prec; ++i) {
    integrator.step();
    for (auto& pos : integrator.positions()) {
      double arg = fmod(pos.arg, numbers::pi) / numbers::pi;
      trace.emplace_back(pos);
    }

    // println();
    double new_best = eval(integrator.best());
    if (new_best < best_score) {
      best_score = new_best;
      fit = integrator.best();
      best_iter = i;
      println("[{}]", i);
    }
  }

  out_file << guesses.size() << endl;
  for (auto& pos : trace) { out_file << pos.mod << " " << pos.arg << endl; }

  println();
  println("    gsa: params: {{{: 3.5f}, {: 3.5f}}}, err^2 = {:.3}, iter = {}",
          fit.mod, fit.arg, best_score, best_iter);
  println("optimal: params: {{{: 3.5f}, {: 3.5f}}}", sqrt(18.), atan2(3, 3));

  system("python3 scripts/plotter.py");
}
