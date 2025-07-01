#include <vector>
#include <queue>
#include <iostream>

struct ConnectNetwork {
    std::vector<int> nodeList; // ノードリスト（例: {0, 0, 1, 0, 0, 0}）
    std::vector<std::vector<int>> edgeList; // エッジ行列（小さいノード番号が行インデックス）
};

// BFSを使用して接続されているノードを探索
std::vector<int> bfs(const std::vector<int>& nodeList, const std::vector<std::vector<int>>& edgeList, int startNode, std::vector<bool>& visited) {
    std::vector<int> connectedNodes(nodeList.size(), 0);
    std::queue<int> queue;

    queue.push(startNode);
    visited[startNode] = true;

    while (!queue.empty()) {
        int currentNode = queue.front();
        queue.pop();
        connectedNodes[currentNode] = 1;

        // 隣接ノードを探索
        for (size_t i = 0; i < edgeList.size(); ++i) {
            if (i < currentNode && edgeList[i][currentNode] == 1 && !visited[i]) {
                queue.push(i);
                visited[i] = true;
            } else if (i > currentNode && edgeList[currentNode][i] == 1 && !visited[i]) {
                queue.push(i);
                visited[i] = true;
            }
        }
    }

    return connectedNodes;
}

// 接続を更新する関数
std::vector<ConnectNetwork> updateConnectNetwork(ConnectNetwork& network, int removedNode1, int removedNode2) {
    // エッジを削除
    if (removedNode1 < removedNode2) {
        network.edgeList[removedNode1][removedNode2] = 0;
    } else {
        network.edgeList[removedNode2][removedNode1] = 0;
    }

    // BFSで接続状態を確認
    std::vector<ConnectNetwork> updatedNetworks;
    std::vector<bool> visited(network.nodeList.size(), false);

    for (size_t i = 0; i < network.nodeList.size(); ++i) {
        if (network.nodeList[i] == 1 && !visited[i]) {
            // BFSで接続されたノードを取得
            std::vector<int> connectedNodes = bfs(network.nodeList, network.edgeList, i, visited);

            // サブグラフを構築
            ConnectNetwork newNetwork;
            newNetwork.nodeList = connectedNodes;

            // エッジ行列を作成
            size_t nodeCount = network.nodeList.size();
            newNetwork.edgeList = std::vector<std::vector<int>>(nodeCount, std::vector<int>(nodeCount, 0));
            for (size_t j = 0; j < nodeCount; ++j) {
                for (size_t k = j + 1; k < nodeCount; ++k) {
                    if (connectedNodes[j] == 1 && connectedNodes[k] == 1) {
                        newNetwork.edgeList[j][k] = network.edgeList[j][k];
                    }
                }
            }

            updatedNetworks.push_back(newNetwork);
        }
    }

    return updatedNetworks;
}

int main() {
    ConnectNetwork network = {
        {1, 1, 1, 1, 1}, // ノードリスト
        {   // エッジ行列（小さいノード番号が行インデックス）
            {0, 1, 0, 0, 0},
            {0, 0, 1, 0, 0},
            {0, 0, 0, 1, 0},
            {0, 0, 0, 0, 1},
            {0, 0, 0, 0, 0}
        }
    };

    // 切断するエッジ
    int removedNode1 = 1;
    int removedNode2 = 2;

    // 更新された接続ネットワークを取得
    auto updatedNetworks = updateConnectNetwork(network, removedNode1, removedNode2);

    // 結果を表示
    for (size_t i = 0; i < updatedNetworks.size(); ++i) {
        std::cout << "ConnectNetwork[" << i << "]:" << std::endl;
        std::cout << "  NodeList: ";
        for (int node : updatedNetworks[i].nodeList) {
            std::cout << node << " ";
        }
        std::cout << std::endl;

        std::cout << "  EdgeList:" << std::endl;
        for (const auto& row : updatedNetworks[i].edgeList) {
            for (int val : row) {
                std::cout << val << " ";
            }
            std::cout << std::endl;
        }
    }

    return 0;
}
