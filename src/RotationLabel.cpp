/*******************************************************************************
 *  TalkingHead - A talking head for robots
 *  Copyright (C) 2012 AG Aktives Sehen <agas@uni-koblenz.de>
 *                     Universitaet Koblenz-Landau
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Library General Public License for more details.
 *
 *  You should have received a copy of the GNU Library General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 *  MA 02110-1301  USA or see <http://www.gnu.org/licenses/>.
 *******************************************************************************/

#include <homer_robot_face/RotationLabel.h>
#include <QPainter>

RotationLabel::RotationLabel( QWidget *parent )
    : QLabel(parent)
{
        this->rotateText(0);
}

RotationLabel::~RotationLabel() { }

bool RotationLabel::rotateText( float degrees )
{
        if (degrees >=0 && degrees <= 360)
        {
                rotation_ = degrees;
                update();
                return true;
        }
        return false;
}

void RotationLabel::paintEvent( QPaintEvent * )
{
        QPainter painter;
        painter.begin( this );
        painter.setFont( font() );
        drawRotatedText( &painter, rotation_, width() , height() , text() );
        painter.end();
}

void RotationLabel::drawRotatedText( QPainter *painter, float degrees, int x, int y, const QString &text )
{
        painter->save();
        QMatrix matrix;
        if( degrees == 0 )
        {
            matrix.translate(0, 0);
            matrix.rotate(degrees);
            painter->setMatrix(matrix);
            painter->drawText( 0, 0, x, y, Qt::AlignCenter | Qt::TextWordWrap, text );
        }
        if( degrees == 90 )
        {
            matrix.translate((width()*4)/5, 0);
            matrix.rotate(degrees);
            painter->setMatrix(matrix);
            painter->drawText( 0, 0, y, x, Qt::AlignCenter | Qt::TextWordWrap, text );
        }
        if( degrees == 180 )
        {
            matrix.translate(width(), height());
            matrix.rotate(degrees);
            painter->setMatrix(matrix);
            painter->drawText( 0, 0, x, y, Qt::AlignCenter | Qt::TextWordWrap, text );
        }
        if( degrees == 270 )
        {
            matrix.translate(0, height());
            matrix.rotate(degrees);
            painter->setMatrix(matrix);
            painter->drawText( 0, 0, y, x, Qt::AlignCenter | Qt::TextWordWrap, text );
        }
        painter->restore();
}
