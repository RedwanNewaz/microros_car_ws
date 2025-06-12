#pragma once 


#include <iostream>
#include <unordered_map>

class TrieNode {
public:
    std::unordered_map<bool, TrieNode*> children;  // Binary children (0 or 1)
    bool isEnd = false;  // Marks the end of a stored key
};

class BinaryTrie {
private:
    TrieNode* root;

    // Encodes (x,y) into a 32-bit integer (16-bit x + 16-bit y)
    uint32_t encode(uint16_t x, uint16_t y) {
        return (static_cast<uint32_t>(x) << 16) | y;
    }

public:
    BinaryTrie() : root(new TrieNode()) {}

    // Inserts (x,y) into the trie
    void insert(uint16_t x, uint16_t y) {
        uint32_t key = encode(x, y);
        TrieNode* node = root;

        for (int i = 31; i >= 0; --i) {
            bool bit = (key >> i) & 1;
            if (node->children.find(bit) == node->children.end()) {
                node->children[bit] = new TrieNode();
            }
            node = node->children[bit];
        }
        node->isEnd = true;
    }

    // Checks if (x,y) exists in the trie (i.e., is occupied)
    bool exists(uint16_t x, uint16_t y) {
        uint32_t key = encode(x, y);
        TrieNode* node = root;

        for (int i = 31; i >= 0; --i) {
            bool bit = (key >> i) & 1;
            if (node->children.find(bit) == node->children.end()) {
                return false;
            }
            node = node->children[bit];
        }
        return node->isEnd;
    }

    // Optional: Destructor to free memory
    ~BinaryTrie() {
        // Recursively delete nodes (implementation omitted for brevity)
    }
};

class OccupancyMap {
private:
    BinaryTrie trie;

public:
    // Marks a cell (x,y) as occupied
    void markOccupied(uint16_t x, uint16_t y) {
        auto xx = ceil(x );
        auto yy = ceil(y );
        trie.insert(xx, yy);
    }

    bool isCollision(uint16_t xx, uint16_t yy, uint16_t radius){
        bool collision = false;
        for(int y = yy - radius; y < yy + radius; y++)
        {
            for(int x = xx - radius; x < xx + radius; x++)
            {
                if(isOccupied(x, y))
                {
                    collision = true;
                    break;
                }
            }
        }

        return collision;
    }

    // Checks if (x,y) is occupied
    bool isOccupied(uint16_t x, uint16_t y) {
        auto xx = ceil(x);
        auto yy = ceil(y);
        return trie.exists(xx, yy);
    }
};