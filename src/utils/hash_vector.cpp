template <typename T>
hash_vector<T>::hash_vector(uint32_t capacity,
                            std::function<uint32_t(const T&)> hash_func,
                            std::function<bool(const T&, const T&)> equal) {
    this->capacity = capacity;
    this->hash_func = hash_func;
    this->equal = equal;
    list.reserve(capacity);
    index_table.resize(capacity);
    for (uint32_t i = 0; i < capacity; ++i) {
        index_table[i] = new link_node();
    }
}

template <typename T>
hash_vector<T>& hash_vector<T>::operator=(const hash_vector<T>& other) {
    list = other.list;
    capacity = other.capacity;
    hash_func = other.hash_func;
    equal = other.equal;
    for (auto p : index_table) {
        while (p != nullptr) {
            auto tmp = p;
            p = p->next;
            delete tmp;
        }
    }
    index_table.resize(capacity);
    for (auto& p : index_table) {
        p = new link_node();
    }
    uint32_t size = list.size();
    for (uint32_t i = 0; i < size; i++) {
        uint32_t idx = hash(list[i]);
        index_table[idx]->next = new link_node(i, index_table[idx]->next);
    }
    return *this;
}

template <typename T>
hash_vector<T>::~hash_vector() {
    for (auto p : index_table) {
        while (p != nullptr) {
            auto tmp = p;
            p = p->next;
            delete tmp;
        }
    }
}

template <typename T>
bool hash_vector<T>::contains(const T& item) const {
    auto p = index_table[hash(item)];
    while (p->next != nullptr) {
        p = p->next;
        if (equal(list[p->val], item)) {
            return true;
        }
    }
    return false;
}

template <typename T>
uint32_t hash_vector<T>::index_of(const T& item) const {
    auto p = index_table[hash(item)];
    while (p->next != nullptr) {
        p = p->next;
        if (equal(list[p->val], item)) {
            return p->val;
        }
    }
    return -1;
}

template <typename T>
bool hash_vector<T>::push_back(const T& item) {
    auto p = index_table[hash(item)];
    while (p->next != nullptr) {
        p = p->next;
        if (equal(list[p->val], item)) {
            return false;
        }
    }
    p->next = new link_node(size());
    list.push_back(item);
    return true;
}

template <typename T>
void hash_vector<T>::pop_back() {
    if (list.empty()) {
        return;
    }
    size_t idx = size() - 1;
    auto p = index_table[hash(list[idx])];
    while (p->next != nullptr && p->next->val != idx) {
        p = p->next;
    }
    auto tmp = p->next;
    p->next = p->next->next;
    delete tmp;
    list.pop_back();
}

template <typename T>
bool hash_vector<T>::insert(const T &item, uint32_t idx) {
    if (idx > size()) {
        return false;
    }
    auto p = index_table[hash(item)];
    while (p->next != nullptr) {
        p = p->next;
        if (equal(list[p->val], item)) {
            return false;
        }
    }
    if (idx == size()) {
        p->next = new link_node(idx);
        list.push_back(item);
        return true;
    }
    for (auto q : index_table) {
        while (q->next != nullptr) {
            q = q->next;
            if (q->val >= idx) {
                q->val++;
            }
        }
    }
    p->next = new link_node(idx);
    list.insert(list.begin() + idx, item);
    return true;
}

template <typename T>
bool hash_vector<T>::append(const hash_vector<T>& other) {
    return append(other.to_vector());
}

template <typename T>
bool hash_vector<T>::append(const std::vector<T>& other) {
    bool flag = true;
    for (auto& item : other) {
        flag &= push_back(item);
    }
    return flag;
}

template <typename T>
bool hash_vector<T>::erase(const T &item) {
    uint32_t idx = index_of(item);
    if (idx == -1) {
        return false;
    }
    erase_at(idx);
    return true;
}

template <typename T>
void hash_vector<T>::erase_at(uint32_t idx) {
    if (idx >= size()) {
        return;
    }
    if (idx == size() - 1) {
        pop_back();
        return;
    }
    for (auto p : index_table) {
        while (p->next != nullptr) {
            if (p->next->val == idx) {
                auto tmp = p->next;
                p->next = p->next->next;
                delete tmp;
                if (p->next == nullptr) {
                    break;
                }
            }
            p = p->next;
            if (p->val > idx) {
                p->val--;
            }
        }
    }
    list.erase(list.begin() + idx);
}

template <typename T>
bool hash_vector<T>::replace(const T& old_item, const T& new_item) {
    uint32_t idx = index_of(old_item);
    if (idx == -1) {
        return false;
    }
    return replace_at(idx, new_item);
}

template <typename T>
bool hash_vector<T>::replace_at(uint32_t idx, const T& new_item) {
    if (idx >= size()) {
        return false;
    }
    auto p = index_table[hash(new_item)];
    while (p->next != nullptr) {
        p = p->next;
        if (equal(list[p->val], new_item)) {
            return false;
        }
    }
    p->next = new link_node(idx);
    p = index_table[hash(list[idx])];
    while (p->next != nullptr) {
        if (p->next->val == idx) {
            auto tmp = p->next;
            p->next = p->next->next;
            delete tmp;
            break;
        }
        p = p->next;
    }
    list[idx] = new_item;
    return true;
}

template <typename T>
void hash_vector<T>::clear() {
    list.clear();
    for (auto& p : index_table) {
        auto q = p->next;
        while (q != nullptr) {
            auto tmp = q;
            q = q->next;
            delete tmp;
        }
        p->next = nullptr;
    }
}
