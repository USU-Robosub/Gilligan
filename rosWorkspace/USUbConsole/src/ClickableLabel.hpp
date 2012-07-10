#ifndef CLICKABLE_LABEL_HPP
#define CLICKABLE_LABEL_HPP

#include <QLabel>

class ClickableLabel : public QLabel
{
    Q_OBJECT
  public:
    ClickableLabel(QWidget* pParent = 0 );
    ~ClickableLabel(){}

    void rectangleDrawState(bool isEnabled);
    void clearRectangle(void);
    short getX1(void);
    short getY1(void);
    short getX2(void);
    short getY2(void);

  signals:
    void clicked();

  public slots:
    void slotClicked() {}

  protected:
    void mousePressEvent(QMouseEvent* pEvent);
    void mouseReleaseEvent(QMouseEvent* pEvent);
    void mouseMoveEvent(QMouseEvent* pEvent);

  private:
    int m_startX;
    int m_startY;
    int m_endX;
    int m_endY;
    bool m_isMousePressed;
    bool m_rectangleDrawingEnabled;
};

#endif // CLICKABLE_LABEL_HPP
