function circle(context, pos, radius, color) {
    context.beginPath();
    context.fillStyle = color;
    context.arc(pos.x, pos.y, radius, 0, Math.PI * 2);
    context.fill();
    context.closePath();
}

class Joystick {
    constructor(x, y, radius, handleRadius) {
        this.pos = new Vector2(x, y); // Current position of the handle 
        this.origin = new Vector2(x, y); // Center position of the joystick
        this.radius = radius;
        this.handleRadius = handleRadius;
        this.ondrag = false;
        this.touchPos = new Vector2(x, y); // Initialize touch position to the center
        this.listener();
    }

    listener() {
        // Touch Events
        addEventListener('touchstart', e => {
            this.touchPos = new Vector2(e.touches[0].pageX, e.touches[0].pageY);
            if (this.touchPos.sub(this.origin).mag() <= this.radius) this.ondrag = true;
        });
        addEventListener('touchend', () => {
            this.ondrag = false;
            this.pos = this.origin;  // Reset the handle position when released
        });
        addEventListener('touchmove', e => {
            this.touchPos = new Vector2(e.touches[0].pageX, e.touches[0].pageY);
            if (this.ondrag) {
                this.reposition();
            }
        });

        // Mouse Events
        addEventListener('mousedown', e => {
            this.touchPos = new Vector2(e.pageX, e.pageY);
            if (this.touchPos.sub(this.origin).mag() <= this.radius) this.ondrag = true;
        });
        addEventListener('mouseup', () => {
            this.ondrag = false;
            this.pos = this.origin;  // Reset the handle position when released
        });
        addEventListener('mousemove', e => {
            this.touchPos = new Vector2(e.pageX, e.pageY);
            if (this.ondrag) {
                this.reposition();
            }
        });
    }

    reposition() {
        if (this.ondrag == false) {
            this.pos = this.pos.add(this.origin.sub(this.pos).mul(this.handleFriction));
        } else {
            const diff = this.touchPos.sub(this.origin);
            const maxDist = Math.min(diff.mag(), this.radius);
            this.pos = this.origin.add(diff.normalize().mul(maxDist));
        }
    }

    draw(context) {
        // Draw Joystick base
        circle(context, this.origin, this.radius, '#707070');
        // Draw Handle when dragging or in default position
        circle(context, this.pos, this.handleRadius, '#3d3d3d');
    }

    update(context) {
        this.draw(context);
        //this.reposition()
    }
}
