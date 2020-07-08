#ifndef MPC_IPOPT_HELPERS_H
#define MPC_IPOPT_HELPERS_H

// Used like this:
// for (auto i : Range(3, 6)) { // Do something }
// This will loop with i = 3, 4, 5
class Range {
    size_t cur;
    const size_t last;
public:
    Range(size_t start, size_t end) : cur(start), last(end) {}

    // Iterable functions
    const Range &begin() const { return *this; }

    const Range &end() const { return *this; }

    // Iterator functions
    bool operator!=(const Range &r) const { return cur < r.last; }

    void operator++() { ++cur; }

    size_t operator*() const { return cur; }

    size_t length() const { return last - cur; }

    friend Range operator+(const Range &a, const Range &b) {
        assert(a.last == b.cur);
        return {a.cur, b.last};
    }
};

// TODO change if not using c++17
template<typename T1, typename T2>
class zip {
    std::pair<T1, T2> containers;
public:
    zip(T1 t1, T2 t2) : containers(std::make_pair(t1, t2)) {}

    auto begin() { return *this; /*std::make_tuple(std::begin(containers[0]), std::begin(containers[1]));*/ }

    auto end() { return *this; /*std::make_tuple(std::end(containers[0]), std::end(containers[1]));*/ }

    bool operator!=(const zip &r) const { return containers.first != r.containers.first; }

    void operator++() {
        ++containers.first;
        ++containers.second;
    }

    auto operator*() const { return std::make_pair(*containers.first, *containers.second); }

};



//    template<typename... T>
//    class zip {
//        std::tuple<T...> containers;
//    public:
//        zip(T &&... containers) : containers(std::make_tuple(containers...)) {}
//
//        auto begin() { return std::make_tuple(std::apply([](auto x) { return std::begin(x); }, containers)); }
//
//        auto end() { return std::make_tuple(std::apply([](auto x) { return std::end(x); }, containers)); }
//
//    };


#endif //MPC_IPOPT_HELPERS_H
