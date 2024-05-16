/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "graphwidget.h"
#include "edge.h"
#include "node.h"
#include <math.h>

#include <QKeyEvent>
#include <QRandomGenerator>
#include <QQueue>
#include <limits>
#include <QVector>
//! [0]
GraphWidget::GraphWidget(QWidget *parent)
    : QGraphicsView(parent), timerId(0)
{
    QGraphicsScene *scene = new QGraphicsScene(this);
    scene->setItemIndexMethod(QGraphicsScene::NoIndex);
    scene->setSceneRect(-200, -200, 400, 400);
    setScene(scene);
    setCacheMode(CacheBackground);
    setViewportUpdateMode(BoundingRectViewportUpdate);
    setRenderHint(QPainter::Antialiasing);
    setTransformationAnchor(AnchorUnderMouse);
    scale(qreal(0.8), qreal(0.8));
    setMinimumSize(400, 400);
    setWindowTitle(tr("Графы"));
//! [0]
    insert = new QPushButton("Добавить");
    delite = new QPushButton("Удалить");
    input = new QPushButton("Решить");
    exit = new QPushButton("Выход");
    new_layout = new QVBoxLayout(this);
    //new_layout->addWidget(insert);
    scene->addWidget(insert);
    scene->addWidget(delite);
    scene->addWidget(input);
    scene->addWidget(exit);
    exit->setGeometry(-200, 150, 100, 50);
    input->setGeometry(-100, 150, 100, 50);
    insert->setGeometry(100, 150, 100, 50);
    delite->setGeometry(0, 150, 100, 50);
//! [1]
    edgeCount = 9;
    node1 = new Node(this, 1);
    node2 = new Node(this, 2);
    node3 = new Node(this, 3);
    node4 = new Node(this, 4);
    node5 = new Node(this, 5);
    node6 = new Node(this, 6);
    graph.resize(vertexCount);

    scene->addItem(node1);
    scene->addItem(node2);
    scene->addItem(node3);
    scene->addItem(node4);
    scene->addItem(node5);
    scene->addItem(node6);
    scene->addItem(new Edge(node1, node2, 12));
    graph[0].push_back({ 1, 12 });
    graph[1].push_back({ 0, 12 });
    scene->addItem(new Edge(node1, node3, 18));
    graph[0].push_back({ 2, 18 });
    graph[2].push_back({ 0, 18 });
    scene->addItem(new Edge(node1, node4, 5));
    graph[0].push_back({ 3, 5 });
    graph[3].push_back({ 0, 5 });
    scene->addItem(new Edge(node1, node6, 34));
    graph[0].push_back({ 5, 34 });
    graph[5].push_back({ 0, 34 });
    scene->addItem(new Edge(node2, node5, 53));
    graph[1].push_back({ 4, 53 });
    graph[4].push_back({ 1, 53 });
    scene->addItem(new Edge(node2, node3, 16));
    graph[1].push_back({ 2, 16 });
    graph[2].push_back({ 1, 16 });
    scene->addItem(new Edge(node2, node6, 45));
    graph[1].push_back({ 5, 45 });
    graph[5].push_back({ 1, 45 });
    scene->addItem(new Edge(node5, node3, 21));
    graph[4].push_back({ 2, 21 });
    graph[2].push_back({ 4, 21 });
    scene->addItem(new Edge(node3, node4, 3));
    graph[2].push_back({ 3, 3 });
    graph[3].push_back({ 2, 3 });
    dijkstra(graph, 2);
    node1->setPos(0, 50);
    node2->setPos(50, -10);
    node3->setPos(-50, 0);
    node4->setPos(50, -50);
    node5->setPos(-50, -50);
    node6->setPos(-50, 50);
    connect(insert, &QPushButton::clicked, this, &GraphWidget::ClickedSlot);
    connect(delite, &QPushButton::clicked, this, &GraphWidget::ClickedSlotRemove);
    connect(exit, &QPushButton::clicked, this, &GraphWidget::Exit);
    connect(input, &QPushButton::clicked, this, &GraphWidget::Answer);

}
//! [1]
void GraphWidget::addVertex(QVector<QVector<QPair<int, int> > > &graph) {
    srand(time(0));
    vertexCount++;

    graph.resize(vertexCount);

    Node *newNode = new Node(this, vertexCount);
    scene()->addItem(newNode);
    int k = rand()%100+1;
    graph[vertexCount-1].push_back({ 0, k });
    graph[0].push_back({ vertexCount-1, k });
    edgeCount++;
    scene()->addItem(new Edge(newNode, node1, k));
    k = rand()%100+1;
    graph[vertexCount-1].push_back({ 2, k });
    graph[2].push_back({ vertexCount-1, k });
    edgeCount++;
    scene()->addItem(new Edge(newNode, node3, k));
    k = rand()%100+1;
    graph[vertexCount-1].push_back({ 3, k });
    graph[3].push_back({ vertexCount-1, k });
    edgeCount++;
    scene()->addItem(new Edge(newNode, node4, k));
    k = rand()%100+1;
    graph[vertexCount-1].push_back({ 4, k });
    graph[4].push_back({ vertexCount-1, k });
    edgeCount++;
    scene()->addItem(new Edge(newNode, node5, k));
    newNode->setPos(-150 + QRandomGenerator::global()->bounded(300), -150 + QRandomGenerator::global()->bounded(300));
    update();
}

void GraphWidget::removeVertex(int vertexIndex) {
    if (vertexIndex < 0 || vertexIndex >= graph.size()) {
        qDebug() << "Invalid vertex index";
        return;
    }

    // Remove all edges connected to the vertex
    for (int i = 0; i < graph[vertexIndex].size(); ++i) {
        int connectedVertex = graph[vertexIndex][i].first;
        for (int j = 0; j < graph[connectedVertex].size(); ++j) {
            if (graph[connectedVertex][j].first == vertexIndex) {
                graph[connectedVertex].remove(j);
                break;
            }
        }
    }

    // Remove the vertex itself
    graph.remove(vertexIndex);

    // Update the indices of the remaining vertices
    for (int i = 0; i < graph.size(); ++i) {
        for (int j = 0; j < graph[i].size(); ++j) {
            if (graph[i][j].first > vertexIndex) {
                graph[i][j].first--;
            }
        }
    }

    // Update the vertexCount
    vertexCount--;

    // Remove the Node item from the scene
    QList<QGraphicsItem *> items = scene()->items();
    for (QGraphicsItem *item : items) {
        Node *node = qgraphicsitem_cast<Node *>(item);
        if (node && node->value == vertexIndex + 1) {
            scene()->removeItem(node);
            delete node;
            break;
        }
    }

    // Remove the Edge items from the scene
    items = scene()->items();
    for (QGraphicsItem *item : items) {
        Edge *edge = qgraphicsitem_cast<Edge *>(item);
        if (edge && (edge->sourceNode()->value == vertexIndex + 1 || edge->destNode()->value == vertexIndex + 1)) {
            scene()->removeItem(edge);
            delete edge;
        }
    }

    // Update the scene
    scene()->update();
}

//! [2]
void GraphWidget::itemMoved()
{
    if (!timerId)
        timerId = startTimer(1000 / 25);
}
//! [2]

//! [3]
void GraphWidget::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_Up:
        centerNode->moveBy(0, -20);
        break;
    case Qt::Key_Down:
        centerNode->moveBy(0, 20);
        break;
    case Qt::Key_Left:
        centerNode->moveBy(-20, 0);
        break;
    case Qt::Key_Right:
        centerNode->moveBy(20, 0);
        break;
    case Qt::Key_Plus:
        zoomIn();
        break;
    case Qt::Key_Minus:
        zoomOut();
        break;
    case Qt::Key_Space:
    case Qt::Key_Enter:
        shuffle();
        break;
    default:
        QGraphicsView::keyPressEvent(event);
    }
}
//! [3]

//! [4]
void GraphWidget::timerEvent(QTimerEvent *event)
{
    Q_UNUSED(event);

    QList<Node *> nodes;
    foreach (QGraphicsItem *item, scene()->items()) {
        if (Node *node = qgraphicsitem_cast<Node *>(item))
            nodes << node;
    }

    foreach (Node *node, nodes)
        node->calculateForces();

    bool itemsMoved = false;
    foreach (Node *node, nodes) {
        if (node->advancePosition())
            itemsMoved = true;
    }

    if (!itemsMoved) {
        killTimer(timerId);
        timerId = 0;
    }
}
//! [4]

#if QT_CONFIG(wheelevent)
//! [5]
void GraphWidget::wheelEvent(QWheelEvent *event)
{
    scaleView(pow((double)2, -event->delta() / 240.0));
}
//! [5]
#endif

//! [6]
void GraphWidget::drawBackground(QPainter *painter, const QRectF &rect)
{
    Q_UNUSED(rect);

    // Shadow
    QRectF sceneRect = this->sceneRect();
    QRectF rightShadow(sceneRect.right(), sceneRect.top() + 5, 5, sceneRect.height());
    QRectF bottomShadow(sceneRect.left() + 5, sceneRect.bottom(), sceneRect.width(), 5);
    if (rightShadow.intersects(rect) || rightShadow.contains(rect))
        painter->fillRect(rightShadow, Qt::darkGray);
    if (bottomShadow.intersects(rect) || bottomShadow.contains(rect))
        painter->fillRect(bottomShadow, Qt::darkGray);

    // Fill
    QLinearGradient gradient(sceneRect.topLeft(), sceneRect.bottomRight());
    gradient.setColorAt(0, Qt::white);
    gradient.setColorAt(1, Qt::lightGray);
    painter->fillRect(rect.intersected(sceneRect), gradient);
    painter->setBrush(Qt::NoBrush);
    painter->drawRect(sceneRect);

    // Text
    QRectF textRect(sceneRect.left() + 4, sceneRect.top() + 4,
                    sceneRect.width() - 4, sceneRect.height() - 4);
    QString message(tr("Задача Коммивояжера"));

    QFont font = painter->font();
    font.setBold(true);
    font.setPointSize(14);
    painter->setFont(font);
    painter->setPen(Qt::lightGray);
    painter->drawText(textRect.translated(2, 2), message);
    painter->setPen(Qt::black);
    painter->drawText(textRect, message);
}
//! [6]

//! [7]
void GraphWidget::scaleView(qreal scaleFactor)
{
    qreal factor = transform().scale(scaleFactor, scaleFactor).mapRect(QRectF(0, 0, 1, 1)).width();
    if (factor < 0.07 || factor > 100)
        return;

    scale(scaleFactor, scaleFactor);
}

void GraphWidget::ClickedSlot()
{
    addVertex(graph);
}

void GraphWidget::ClickedSlotRemove()
{
    if (vertexCount > 0) {
            removeVertex(vertexCount - 1);
    }
}

void GraphWidget::Exit()
{
    close();
    close();
}

void GraphWidget::Answer()
{
//    for (int i = 0; i < graph.size(); i++) {
//        qDebug() << "Вершина " << i + 1 << ": ";
//        for (int j = 0; j < graph[i].size(); j++) {
//            qDebug() << "связана с вершиной" << graph[i][j].first + 1 << "с весом ребра" << graph[i][j].second;
//        }
//        qDebug() << endl;
//    }
    dijkstra(graph, 2);
}

//! [7]

void GraphWidget::shuffle()
{
    foreach (QGraphicsItem *item, scene()->items()) {
        if (qgraphicsitem_cast<Node *>(item))
            item->setPos(-150 + QRandomGenerator::global()->bounded(300), -150 + QRandomGenerator::global()->bounded(300));
    }
}

void GraphWidget::zoomIn()
{
    scaleView(qreal(1.2));
}

void GraphWidget::zoomOut()
{
    scaleView(1 / qreal(1.2));
}
const int INF = std::numeric_limits<int>::max();

void GraphWidget::dijkstra(const QVector<QVector<QPair<int, int>>> &graph, int start) {
    QVector<int> dist(graph.size(), INF);
    QQueue<QPair<int, int>> pq;

    dist[start] = 0;
    pq.enqueue({0, start});

    while (!pq.isEmpty()) {
        int v = pq.head().second;
        int current_dist = pq.head().first;
        pq.dequeue();

        if (current_dist > dist[v])
            continue;

        for (const QPair<int, int> &edge : graph[v]) {
            int to = edge.first;
            int weight = edge.second;

            if (dist[v] + weight < dist[to]) {
                dist[to] = dist[v] + weight;
                pq.enqueue({dist[to], to});
            }
        }

        // Использование лямбда-выражения для сравнения элементов
        std::sort(pq.begin(), pq.end(), [](const QPair<int, int>& a, const QPair<int, int>& b) {
            return a.first < b.first;
        });
    }

    for (int i = 0; i < dist.size(); ++i) {
        if (dist[i] != INF)
            qDebug() << "Вершина" << i + 1 << ":" << dist[i];
        else
            qDebug() << "Вершина" << i + 1 << ":" << "недостижима";
    }
    qDebug() << endl;
}
