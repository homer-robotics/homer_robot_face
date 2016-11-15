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

#ifndef TALKING_HEAD_INCLUDE_ROTATIONLABEL_H_
#define TALKING_HEAD_INCLUDE_ROTATIONLABEL_H_

#include <QLabel>

/**
 * @class  RotationLabel
 * @brief  Handles rotation of label.
 * @author Julian Giesen (R16)(R18)
 */

class RotationLabel : public QLabel{
        Q_OBJECT

public:
        explicit RotationLabel( QWidget *parent = 0 );
        virtual ~RotationLabel();

        bool rotateText( float degrees );

private:
        float rotation_;

protected:
        void paintEvent( QPaintEvent * );
        void drawRotatedText( QPainter *painter, float degrees, int x, int y, const QString &text );
};

#endif // TALKING_HEAD_INCLUDE_ROTATIONLABEL_H_
