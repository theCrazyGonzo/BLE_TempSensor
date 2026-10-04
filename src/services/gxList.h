#pragma once
#include <vector>
#include <stdexcept>
#include <cstdint>
#include <algorithm>

template<typename T>
class gxList {
private:
    std::vector<T> data;

public:
    // Konstruktor
    gxList() = default;

    // Destruktor
    ~gxList() = default;

    // Copy Constructor
    gxList(const gxList& other) = default;

    // Move Constructor
    gxList(gxList&& other) noexcept = default;

    // Copy Assignment
    gxList& operator=(const gxList& other) = default;

    // Move Assignment
    gxList& operator=(gxList&& other) noexcept = default;

    // -------------------------------------------------------------------------
    // Iterator-Support (delegiert direkt an std::vector)
    // Ermöglicht range-based for-Loops und STL-Algorithmen
    // -------------------------------------------------------------------------

    using iterator               = typename std::vector<T>::iterator;
    using const_iterator         = typename std::vector<T>::const_iterator;
    using reverse_iterator       = typename std::vector<T>::reverse_iterator;
    using const_reverse_iterator = typename std::vector<T>::const_reverse_iterator;

    // Vorwärts-Iteratoren
    iterator               begin()        { return data.begin(); }
    iterator               end()          { return data.end();   }
    const_iterator         begin()  const { return data.begin(); }
    const_iterator         end()    const { return data.end();   }
    const_iterator         cbegin() const { return data.cbegin(); }
    const_iterator         cend()   const { return data.cend();   }

    // Rückwärts-Iteratoren
    reverse_iterator       rbegin()        { return data.rbegin(); }
    reverse_iterator       rend()          { return data.rend();   }
    const_reverse_iterator rbegin()  const { return data.rbegin(); }
    const_reverse_iterator rend()    const { return data.rend();   }
    const_reverse_iterator crbegin() const { return data.crbegin(); }
    const_reverse_iterator crend()   const { return data.crend();   }

    // -------------------------------------------------------------------------

    // Fügt einen Wert am Ende der Liste hinzu
    void add(const T& value) {
        data.push_back(value);
    }

    // Überladung für Move-Semantik (C++20)
    //
    // list.addUnique(std::move(s));
    // variable s ist dann verschoben und in funktion leer, aber vorhanden
    void add(T&& value) {
        data.push_back(std::move(value));
    }

    bool addUnique(const T& value) {
        if (contains(value))
            return false;  // bereits vorhanden
        data.push_back(value);
        return true;
    }

    bool addUnique(T&& value) {
        if (contains(value))
            return false;  // bereits vorhanden
        data.push_back(std::move(value));
        return true;
    }

    // Gibt den Wert an der angegebenen Position zurück
    T get(uint16_t index) const {
        if (index >= data.size()) {
            throw std::out_of_range("gxList: index out of range");
        }
        return data[index];
    }

    // Referenz-Version für Änderungen (optional)
    T& get(uint16_t index) {
        if (index >= data.size()) {
            throw std::out_of_range("gxList: index out of range");
        }
        return data[index];
    }

    bool contains(const T& value) const {
        for (const auto& item : data) {
            if (item == value)
                return true;
        }
        return false;
    }

    // Gibt die Anzahl der Elemente in der Liste zurück
    // uint16_t getSize() const {
    //     return static_cast<uint16_t>(data.size());
    // }

    uint16_t size() const {
        return static_cast<uint16_t>(data.size());
    }

    // Fügt ein Element an der angegebenen Position ein
    void insert(uint16_t index, const T& value) {
        if (index > data.size()) {
            throw std::out_of_range("gxList: index out of range");
        }
        data.insert(data.begin() + index, value);
    }

    // Überladung für Move-Semantik (C++20)
    void insert(uint16_t index, T&& value) {
        if (index > data.size()) {
            throw std::out_of_range("gxList: index out of range");
            return;
        }
        data.insert(data.begin() + index, std::move(value));
    }

    // Löscht das Element an der angegebenen Position
    void remove(uint16_t index) {
        if (index >= data.size()) {
            throw std::out_of_range("gxList: index out of range");
            return;
        }
        data.erase(data.begin() + index);
    }

    void removeAndDelete(uint16_t index) {
        if (index >= data.size()) {
            throw std::out_of_range("gxList: index out of range");
            return;
        }
        delete data[index];  // Nur für T = Pointer-Typ!
        data.erase(data.begin() + index);
    }

    bool removeByValue(const T& value) {
        for (auto it = data.begin(); it != data.end(); ++it) {
            if (*it == value) {
                data.erase(it);
                return true;  // found and erased
            }
        }
        return false;  // not found
    }

    uint16_t removeAllByValue(const T& value) {
        size_t before = data.size();
        data.erase(std::remove(data.begin(), data.end(), value), data.end());
        return static_cast<uint16_t>(before - data.size());  // count of erased entries
    }

    void swap(uint16_t index1, uint16_t index2) {
        if (index1 >= data.size() || index2 >= data.size()) {
            throw std::out_of_range("gxList: index out of range");
        }
        std::swap(data[index1], data[index2]);
    }

    bool empty() const {
        return data.empty();
    }

    void clear() {
        data.clear();
    }
};


// -------------------------------------------------------------------------
// Verwendungsbeispiele für die Iteration:
// -------------------------------------------------------------------------
/*
#include <iostream>
#include <algorithm>  // std::find, std::sort, ...

int main() {
    gxList<int> list;
    list.add(10);
    list.add(20);
    list.add(30);

    // 1) Range-based for (lesend)
    for (const auto& val : list) {
        std::cout << val << "\n";
    }

    // 2) Range-based for (schreibend)
    for (auto& val : list) {
        val *= 2;
    }

    // 3) Klassischer Iterator
    for (auto it = list.begin(); it != list.end(); ++it) {
        std::cout << *it << "\n";
    }

    // 4) Rückwärts iterieren
    for (auto it = list.rbegin(); it != list.rend(); ++it) {
        std::cout << *it << "\n";
    }

    // 5) STL-Algorithmen
    auto it = std::find(list.begin(), list.end(), 40);
    if (it != list.end()) {
        std::cout << "Gefunden: " << *it << "\n";
    }

    return 0;
}
*/
