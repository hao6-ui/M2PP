import copy
import random
from math import inf


class Tree:
    def __init__(self, node) -> None:
        self.root = node
        self.high = 1
        self.node = []

    def addNode(self, node1, node2) -> None:
        compareResult = self.compareNode(node1, node2)

        if compareResult == -1:
            # 没有重叠的情况
            left = node1.left if node1.left[0] < node2.left[0] else node2.left
            right = node1.right if node1.right[0] > node2.right[0] else node2.right
            bottom = node1.bottom if node1.bottom[1] < node2.bottom[1] else node2.bottom
            top = node1.top if node1.top[1] > node2.top[1] else node2.top

            newNode = TreeNode([left, right, bottom, top], node2.parent, [node1, node2])

            self.high += 1
            if node2.parent is not None:
                node2.parent.children.remove(node2)
            node2.parent = newNode
            node1.parent = newNode

            # 根节点处理
            if newNode.parent == None:
                self.root = newNode
            return
        elif compareResult == 1:
            # 包含的情况
            self.addNodeByLevel(node1, node2)
            if node2.left[0] < node2.left[1] and node2.bottom[1] < node2.top[1]:
                node1.children.append(node2)
                node2.parent = node1
            return
        else:
            # 被包含的情况
            # minHeight = inf
            # minNode = ''
            nodeList = []
            for node in node1.children:
                levelResult = self.addNodeByLevel(node, node2)
                if levelResult == -1:
                    continue
                elif levelResult == 1:
                    # nodeHeight = self.getMaxDepth(node)
                    # if nodeHeight < minHeight:
                    #     minNode = node
                    #     minHeight = nodeHeight

                    nodeList.append(node)
                    # self.addNode(node, node2)
                    # return

            # if minHeight != inf:
            #     self.addNode(minNode, node2)
            #     return
            if len(nodeList) > 0:
                nodeIndex = random.randint(0, len(nodeList) - 1)
                self.addNode(nodeList[nodeIndex], node2)
                return

            if node2.left[0] < node2.left[1] and node2.bottom[1] < node2.top[1]:
                node1.children.append(node2)
                node2.parent = node1
            return

    # 每层节点依次比较等待插入节点
    def addNodeByLevel(self, node1, node2):
        compareResult = self.compareNode(node1, node2)
        if compareResult == -1:  # 不重叠返回原节点
            return -1
        elif compareResult == 1:  # 包含起来
            return 1
        else:  # 重叠进行切割
            if node2.left[0] > node1.left[0] and node2.left[0] < node1.right[0] and node2.right[0] > node1.right[0]:
                node2.left[0] = node1.right[0]
            elif node2.right[0] > node1.left[0] and node2.right[0] < node1.right[0] and node2.left[0] < node2.left[0]:
                node2.right[0] = node1.left[0]

            if node2.top[1] > node1.bottom[1] and node2.top[1] < node1.top[1] and node2.bottom[1] < node1.bottom[1]:
                node2.top[1] = node1.bottom[1]
            elif node2.bottom[1] > node1.bottom[1] and node2.bottom[1] < node1.top[0] and node2.top[1] < node2.top[1]:
                node2.bottom[1] = node1.top[1]

            return 0

    # 随机获取叶节点
    def getNode(self, node, h, g):
        if (node and len(node.children) == 0) or h == g:
            if node.parent is not None:
                self.node = [node.bottom, node.top, node.parent.bottom, node.parent.top]
            else:
                self.node = [node.bottom, node.top]
            return
        x = random.randint(0, len(node.children) - 1)
        self.getNode(node.children[x], h, g + 1)

    # 遍历树
    def viewTree(self, node, c):
        print(c, [node.left, node.right, node.bottom, node.top])
        if len(node.children) == 0:
            return
        for x in node.children:
            self.viewTree(x, c + 1)

    # 计算区域是否重叠
    def compareNode(self, node1, node2):

        # -1没有重叠
        if node1.right[0] <= node2.left[0] or node1.left[0] >= node2.right[0] or node1.top[1] <= node2.bottom[1] or \
                node1.bottom[1] >= node2.top[1]:
            return -1

        # 1被包含
        if node2.left[0] > node1.left[0] and node2.right[0] < node1.right[0] and node2.top[1] < node1.top[1] and \
                node2.bottom[1] > node1.bottom[1]:
            return 1

        # 0为部分重叠
        return 0

    # 获取树节点最大深度
    # def getMaxDepth(self, node):
    #     if len(node.children) == 0:
    #         return 0
    #     ans = 0
    #     for child in node.children:
    #         ans = max(ans, 1 + self.getMaxDepth(child))
    #     return ans


class TreeNode:
    def __init__(self, val, parent=None, children=[]):
        self.children = children
        self.parent = parent
        self.left = val[0]
        self.right = val[1]
        self.bottom = val[2]
        self.top = val[3]
