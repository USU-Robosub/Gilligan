#include <QMouseEvent>
#include <QPoint>
#include <QPainter>
#include <stdio.h>

#include "ClickableLabel.hpp"

ClickableLabel::ClickableLabel(QWidget* pParent)
  : QLabel(pParent),
    m_startX(0),
    m_startY(0),
    m_endX(0),
    m_endY(0),
    m_isMousePressed(false),
    m_rectangleDrawingEnabled(false)
{
    this->setAttribute(Qt::WA_PaintOutsidePaintEvent);
    connect(this, SIGNAL(clicked()), this, SLOT(slotClicked()));
}

void ClickableLabel::rectangleDrawState(bool isEnabled)
{
    m_rectangleDrawingEnabled = isEnabled;
}

void ClickableLabel::mousePressEvent(QMouseEvent* pEvent)
{
    QPainter painter(this);
    painter.eraseRect(0, 0, this->width(), this->height());

    if (m_rectangleDrawingEnabled)
    {
        m_startX = pEvent->pos().x();
        m_startY = pEvent->pos().y();
    }
}

void ClickableLabel::mouseReleaseEvent(QMouseEvent* pEvent)
{
    m_endX = pEvent->pos().x();
    m_endY = pEvent->pos().y();

    if (m_rectangleDrawingEnabled)
    {
        QPainter painter(this);
        painter.eraseRect(0, 0, this->width(), this->height());
        painter.setPen(Qt::yellow);
        painter.drawRect(m_startX, m_startY, m_endX - m_startX, m_endY - m_startY);

        emit clicked();
    }
}

void ClickableLabel::mouseMoveEvent(QMouseEvent* pEvent)
{
    m_endX = pEvent->pos().x();
    m_endY = pEvent->pos().y();

    if (m_rectangleDrawingEnabled)
    {
        QPainter painter(this);
        painter.eraseRect(0, 0, this->width(), this->height());
        painter.setPen(Qt::yellow);
        painter.drawRect(m_startX, m_startY, pEvent->pos().x() - m_startX, pEvent->pos().y() - m_startY);
     }
}

short ClickableLabel::getX1(void)
{
    return m_startX - 239;
}

short ClickableLabel::getY1(void)
{
    return -(m_startY - 319);
}

short ClickableLabel::getX2(void)
{
    return m_endX - 239;
}

short ClickableLabel::getY2(void)
{
    return -(m_endY - 319);
}

void ClickableLabel::clearRectangle(void)
{
    QPainter painter(this);
    painter.eraseRect(0, 0, this->width(), this->height());
}
