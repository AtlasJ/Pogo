#include "AlgoH3GLSurface.h"

#include <QCoreApplication>
#include <QOpenGLContext>
#include <QOpenGLFramebufferObject>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QOffscreenSurface>
#include <QMatrix4x4>
#include <QVector3D>
#include <QVector4D>
#include <QSurfaceFormat>
#include <QThread>
#include <QtMath>
#include <algorithm>
#include <cmath>
#include <limits>

namespace {

const float kNaNf = std::numeric_limits<float>::quiet_NaN();
inline bool validZ(float v) { return v == v; }

//the same light and ambient floor the CPU renderer uses, so the two agree on which way a flank
//faces; see kMeshLight* in AlgoHeight3.cpp
const float kLightX = -0.3990f, kLightY = -0.6484f, kLightZ = 0.6484f;
const float kAmbient = 0.32f;
const float kBgR = 24 / 255.0f, kBgG = 26 / 255.0f, kBgB = 32 / 255.0f;

/*
* Vertex: model position, the grid slope at that node, and a grey level for the textured style.
*
* The slope rather than a finished normal, because the Z exaggeration is applied in the shader -
* a normal baked on the CPU would be right for one exaggeration only, and the operator drags
* that slider while watching the surface.
*/
struct Vtx { float x, y, z, gx, gy, t; };

const char* kVS = R"(#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec2 aGrad;
layout(location = 2) in float aTex;
uniform mat4 uMVP;
uniform mat3 uNormal;     // the same rotation applied to the position, for view-space lighting
uniform float uZSpan;     // model half-height: the colour parameter is recovered from it
out float vT;             // 0..1 height, for the colour ramp
out float vTex;
out vec3 vN;
void main() {
	gl_Position = uMVP * vec4(aPos, 1.0);
	vT = aPos.z / max(2.0 * uZSpan, 1e-6) + 0.5;
	vTex = aTex;
	//the surface normal of z = f(x,y) is (-df/dx, -df/dy, 1); the slope arrives per unit of
	//model x/y already, so only the rotation is left to apply
	vN = uNormal * normalize(vec3(-aGrad.x, -aGrad.y, 1.0));
})";

const char* kFS = R"(#version 330 core
in float vT;
in float vTex;
in vec3 vN;
uniform sampler2D uRamp;
uniform int uMode;        // 0 ramp by height, 1 grey from the texture, 2 flat uColor
uniform int uLit;
uniform vec3 uLight;
uniform float uAmbient;
uniform vec4 uColor;
uniform float uDarken;    // 1.0 normal; < 1 for the mesh style's own darker edge lines
out vec4 frag;
void main() {
	vec3 c;
	if (uMode == 0)      c = texture(uRamp, vec2(clamp(vT, 0.0, 1.0), 0.5)).rgb;
	else if (uMode == 1) c = vec3(vTex);
	else                 c = uColor.rgb;

	if (uLit != 0) {
		//fabs, not max(0, ...): these facets are never back-face culled, so one turned away
		//from the light must still be shaded rather than dropping to pure ambient and
		//punching a hole in the surface
		float d = abs(dot(normalize(vN), uLight));
		c *= uAmbient + (1.0 - uAmbient) * min(1.0, d);
	}
	frag = vec4(c * uDarken, 1.0);
})";

/*
* The GL context, its surface and the shaders, made once and kept.
*
* Thread-affine on purpose: a QOpenGLContext belongs to the thread that made it, so the owning
* thread is recorded and a call from anywhere else is refused rather than silently corrupting
* the context. The page renders on the GUI thread, which is the only caller today.
*/
class GLBackend
{
public:
	static GLBackend& instance() { static GLBackend b; return b; }

	bool ready()
	{
		if (m_tried) return m_ok && m_thread == QThread::currentThread();
		m_tried = true;
		m_thread = QThread::currentThread();
		m_ok = init();
		return m_ok;
	}

	QString error() const { return m_error; }

	QImage render(const AlgoH3GLScene& scene, double yawDeg, double pitchDeg,
		double zExaggeration, const QSize& outSize, AlgoH3SurfaceStyle style);

private:
	GLBackend() = default;
	//deliberately NOT destroy(): this is a function-local static, so it would run after the
	//QApplication is gone and makeCurrent() on a dead context is a crash. The process is on its
	//way out and the driver reclaims everything anyway.
	~GLBackend() = default;
	GLBackend(const GLBackend&) = delete;
	GLBackend& operator=(const GLBackend&) = delete;

	bool init();
	void destroy();
	void uploadRamp(const unsigned int* ramp);
	bool ensureFbo(const QSize& size);

	bool m_tried = false, m_ok = false;
	QString m_error;
	QThread* m_thread = nullptr;

	QOpenGLContext* m_ctx = nullptr;
	QOffscreenSurface* m_surface = nullptr;
	QOpenGLFunctions_3_3_Core* f = nullptr;
	QOpenGLShaderProgram* m_prog = nullptr;
	QOpenGLFramebufferObject* m_fbo = nullptr;
	GLuint m_vao = 0, m_vbo = 0, m_ebo = 0, m_rampTex = 0;
	const unsigned int* m_rampSrc = nullptr;
};

bool GLBackend::init()
{
	if (!QCoreApplication::instance()) { m_error = QStringLiteral("no QApplication"); return false; }

	QSurfaceFormat fmt;
	fmt.setRenderableType(QSurfaceFormat::OpenGL);
	fmt.setVersion(3, 3);
	fmt.setProfile(QSurfaceFormat::CoreProfile);
	fmt.setDepthBufferSize(24);

	m_ctx = new QOpenGLContext;
	m_ctx->setFormat(fmt);
	if (!m_ctx->create()) {
		m_error = QStringLiteral("could not create an OpenGL context");
		destroy(); return false;
	}
	//a context can come back older than asked for; 3.3 core is what the shaders need
	const QSurfaceFormat got = m_ctx->format();
	if (got.majorVersion() * 10 + got.minorVersion() < 33
		|| got.renderableType() != QSurfaceFormat::OpenGL) {
		m_error = QStringLiteral("OpenGL %1.%2 - need desktop GL 3.3 core (set "
			"Qt::AA_UseDesktopOpenGL before the QApplication)")
			.arg(got.majorVersion()).arg(got.minorVersion());
		destroy(); return false;
	}

	m_surface = new QOffscreenSurface;
	m_surface->setFormat(m_ctx->format());
	m_surface->create();
	if (!m_surface->isValid() || !m_ctx->makeCurrent(m_surface)) {
		m_error = QStringLiteral("could not make the offscreen surface current");
		destroy(); return false;
	}

	f = m_ctx->versionFunctions<QOpenGLFunctions_3_3_Core>();
	if (!f || !f->initializeOpenGLFunctions()) {
		m_error = QStringLiteral("OpenGL 3.3 core functions are unavailable");
		destroy(); return false;
	}

	m_prog = new QOpenGLShaderProgram;
	if (!m_prog->addShaderFromSourceCode(QOpenGLShader::Vertex, kVS)
		|| !m_prog->addShaderFromSourceCode(QOpenGLShader::Fragment, kFS)
		|| !m_prog->link()) {
		m_error = QStringLiteral("shader build failed: ") + m_prog->log();
		destroy(); return false;
	}

	f->glGenVertexArrays(1, &m_vao);
	f->glGenBuffers(1, &m_vbo);
	f->glGenBuffers(1, &m_ebo);
	f->glBindVertexArray(m_vao);
	f->glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	f->glEnableVertexAttribArray(0);
	f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vtx), (void*)0);
	f->glEnableVertexAttribArray(1);
	f->glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Vtx), (void*)(3 * sizeof(float)));
	f->glEnableVertexAttribArray(2);
	f->glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(Vtx), (void*)(5 * sizeof(float)));
	f->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);   //the index binding belongs to the VAO
	f->glBindVertexArray(0);

	f->glGenTextures(1, &m_rampTex);
	m_ctx->doneCurrent();
	return true;
}

void GLBackend::destroy()
{
	if (m_ctx && m_surface && m_ctx->makeCurrent(m_surface)) {
		if (f) {
			if (m_vbo) f->glDeleteBuffers(1, &m_vbo);
			if (m_ebo) f->glDeleteBuffers(1, &m_ebo);
			if (m_vao) f->glDeleteVertexArrays(1, &m_vao);
			if (m_rampTex) f->glDeleteTextures(1, &m_rampTex);
		}
		delete m_fbo; m_fbo = nullptr;
		delete m_prog; m_prog = nullptr;
		m_ctx->doneCurrent();
	}
	delete m_surface; m_surface = nullptr;
	delete m_ctx; m_ctx = nullptr;
	f = nullptr;
	m_vao = m_vbo = m_ebo = m_rampTex = 0;
}

void GLBackend::uploadRamp(const unsigned int* ramp)
{
	if (!ramp || ramp == m_rampSrc) return;
	m_rampSrc = ramp;
	unsigned char px[256 * 3];
	for (int i = 0; i < 256; i++) {
		const unsigned int c = ramp[i];          //0xAARRGGBB
		px[i * 3 + 0] = (unsigned char)((c >> 16) & 0xFF);
		px[i * 3 + 1] = (unsigned char)((c >> 8) & 0xFF);
		px[i * 3 + 2] = (unsigned char)(c & 0xFF);
	}
	f->glBindTexture(GL_TEXTURE_2D, m_rampTex);
	f->glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	f->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 256, 1, 0, GL_RGB, GL_UNSIGNED_BYTE, px);
	f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	f->glBindTexture(GL_TEXTURE_2D, 0);
}

bool GLBackend::ensureFbo(const QSize& size)
{
	if (m_fbo && m_fbo->size() == size) return true;
	delete m_fbo;
	QOpenGLFramebufferObjectFormat fmt;
	fmt.setAttachment(QOpenGLFramebufferObject::Depth);
	//multisampling instead of the CPU path's hard aliased edges; a surface made of thousands of
	//small facets shows its stair-stepping badly without it
	fmt.setSamples(4);
	m_fbo = new QOpenGLFramebufferObject(size, fmt);
	if (m_fbo->isValid()) return true;

	delete m_fbo;
	fmt.setSamples(0);                       //some drivers refuse a multisampled FBO this large
	m_fbo = new QOpenGLFramebufferObject(size, fmt);
	return m_fbo->isValid();
}

QImage GLBackend::render(const AlgoH3GLScene& scene, double yawDeg, double pitchDeg,
	double zExaggeration, const QSize& outSize, AlgoH3SurfaceStyle style)
{
	const int gw = scene.w, gh = scene.h;
	if (gw < 2 || gh < 2 || (long long)scene.z.size() < (long long)gw * gh) return QImage();
	if (!m_ctx->makeCurrent(m_surface)) return QImage();
	if (!ensureFbo(outSize)) { m_ctx->doneCurrent(); return QImage(); }

	// ── model space: identical to the CPU renderer, so the two frame the part the same ──
	const double aspect = (double)gw / (double)std::max(1, gh);
	const double ax = (aspect >= 1.0) ? 1.0 : aspect;
	const double ay = (aspect >= 1.0) ? 1.0 / aspect : 1.0;
	const double zSpan = std::max(0.05, std::min(2.0, zExaggeration)) * 0.5;
	const double zRange = (scene.zMax > scene.zMin) ? (scene.zMax - scene.zMin) : 1.0;

	const bool textured = (style == AlgoH3SurfaceStyle::Textured) && !scene.tex.empty();
	const bool points = (style == AlgoH3SurfaceStyle::PointCloud);
	const bool wire = (style == AlgoH3SurfaceStyle::Wireframe);
	const bool meshLines = (style == AlgoH3SurfaceStyle::ShadedMesh);
	const bool lit = (style != AlgoH3SurfaceStyle::Filled && !wire && !points);

	// ── the projection the CPU renderer uses, written as a matrix ──
	//Spin about the vertical axis, then tilt the whole thing toward the viewer - the CPU
	//renderer's projection, written as a matrix.
	//
	//Row 1 is negated because its screen y grows DOWNWARD while clip space's grows upward.
	//Row 2 is NOT: like the CPU renderer's depth it grows toward the viewer, which is what
	//GL's eye space wants as well - the eye looks down -z there, so a larger z is nearer.
	//The ortho near/far below are negated to match. Get this backwards and the substrate
	//wins the depth test against the pins standing on it.
	const double yaw = qDegreesToRadians(yawDeg);
	const double elev = qDegreesToRadians(std::max(2.0, std::min(89.0, pitchDeg)));
	const double cy = std::cos(yaw), sy = std::sin(yaw);
	const double ce = std::cos(elev), se = std::sin(elev);

	const double r00 = cy,       r01 = -sy,      r02 = 0.0;
	const double r10 = -sy * se, r11 = -cy * se, r12 = ce;
	const double r20 = sy * ce,  r21 = cy * ce,  r22 = se;

	QMatrix4x4 rot;
	rot.setRow(0, QVector4D((float)r00, (float)r01, (float)r02, 0.0f));
	rot.setRow(1, QVector4D((float)r10, (float)r11, (float)r12, 0.0f));
	rot.setRow(2, QVector4D((float)r20, (float)r21, (float)r22, 0.0f));
	rot.setRow(3, QVector4D(0.0f, 0.0f, 0.0f, 1.0f));

	// ── vertices, and the rotated bounds in the same pass ──
	//Fitting from the measured vertices rather than from the model box, because a part that
	//only occupies a corner of its own bounding box should still fill the view - and in one
	//pass, because a second walk over a million vertices is what a drag cannot afford.
	std::vector<Vtx> verts((size_t)gw * gh);
	double minX = 1e18, maxX = -1e18, minY = 1e18, maxY = -1e18, minZ = 1e18, maxZ = -1e18;
	const double dx = (gw > 1) ? 2.0 * ax / (gw - 1) : 1.0;   //model units per cell
	const double dy = (gh > 1) ? 2.0 * ay / (gh - 1) : 1.0;

	auto tAt = [&](int i, int j) -> float {
		if (i < 0 || j < 0 || i >= gw || j >= gh) return kNaNf;
		const float v = scene.z[(size_t)j * gw + i];
		return validZ(v) ? (float)((v - scene.zMin) / zRange) : kNaNf;
	};

	for (int j = 0; j < gh; j++) {
		for (int i = 0; i < gw; i++) {
			Vtx& v = verts[(size_t)j * gw + i];
			const float t = tAt(i, j);
			v.x = (float)(2.0 * i / (double)(gw - 1) - 1.0) * (float)ax;
			v.y = (float)(2.0 * j / (double)(gh - 1) - 1.0) * (float)ay;
			//the shader reads z back out as the 0..1 colour parameter, so the exaggeration is
			//folded in here rather than in the matrix
			v.z = validZ(t) ? (float)((t - 0.5) * 2.0 * zSpan) : 0.0f;

			//central difference where both sides are measured, one-sided where only one is
			double gx = 0.0, gy = 0.0;
			if (validZ(t)) {
				const float l = tAt(i - 1, j), r = tAt(i + 1, j);
				const float u = tAt(i, j - 1), d = tAt(i, j + 1);
				const double k = 2.0 * zSpan;
				if (validZ(l) && validZ(r))      gx = (r - l) * k / (2 * dx);
				else if (validZ(r))              gx = (r - t) * k / dx;
				else if (validZ(l))              gx = (t - l) * k / dx;
				if (validZ(u) && validZ(d))      gy = (d - u) * k / (2 * dy);
				else if (validZ(d))              gy = (d - t) * k / dy;
				else if (validZ(u))              gy = (t - u) * k / dy;
			}
			v.gx = (float)gx; v.gy = (float)gy;
			v.t = textured ? scene.tex[(size_t)j * gw + i] / 255.0f : 0.0f;

			if (!validZ(t)) continue;   //an unmeasured node is drawn by nothing, so it frames nothing
			const double px = r00 * v.x + r01 * v.y + r02 * v.z;
			const double py = r10 * v.x + r11 * v.y + r12 * v.z;
			const double pz = r20 * v.x + r21 * v.y + r22 * v.z;
			minX = std::min(minX, px); maxX = std::max(maxX, px);
			minY = std::min(minY, py); maxY = std::max(maxY, py);
			minZ = std::min(minZ, pz); maxZ = std::max(maxZ, pz);
		}
	}

	// ── indices: a quad with an unmeasured corner yields what triangles it can ──
	std::vector<unsigned int> idx;
	std::vector<unsigned int> pts;
	if (points) {
		pts.reserve((size_t)gw * gh);
		for (int j = 0; j < gh; j++) for (int i = 0; i < gw; i++)
			if (validZ(tAt(i, j))) pts.push_back((unsigned int)((size_t)j * gw + i));
		if (pts.empty()) { m_ctx->doneCurrent(); return QImage(); }
	}
	else {
		idx.reserve((size_t)(gw - 1) * (gh - 1) * 6);
		auto ok = [&](int i, int j) { return validZ(tAt(i, j)); };
		for (int j = 0; j + 1 < gh; j++) {
			for (int i = 0; i + 1 < gw; i++) {
				const unsigned int a = (unsigned int)((size_t)j * gw + i), b = a + 1;
				const unsigned int c = a + gw, d = c + 1;
				const bool oa = ok(i, j), ob = ok(i + 1, j), oc = ok(i, j + 1), od = ok(i + 1, j + 1);
				//the whole cell, or - where a shadow clips one corner - the one triangle that is
				//still fully measured, so the surface erodes a corner at a time instead of
				//losing a whole square
				if (oa && ob && oc && od) {
					idx.insert(idx.end(), { a, c, d, a, d, b });
				}
				else if (oa && oc && od) idx.insert(idx.end(), { a, c, d });
				else if (oa && od && ob) idx.insert(idx.end(), { a, d, b });
				else if (oa && oc && ob) idx.insert(idx.end(), { a, c, b });
				else if (ob && oc && od) idx.insert(idx.end(), { b, c, d });
			}
		}
		if (idx.empty()) { m_ctx->doneCurrent(); return QImage(); }
	}

	if (maxX <= minX || maxY <= minY) { m_ctx->doneCurrent(); return QImage(); }

	// ── fit the rotated content to the canvas, whatever the angle ──
	const double margin = 18.0;
	const double W = outSize.width(), H = outSize.height();
	const double s = std::min((W - 2 * margin) / (maxX - minX), (H - 2 * margin) / (maxY - minY));
	const double halfW = W / (2.0 * s), halfH = H / (2.0 * s);
	const double cxw = 0.5 * (minX + maxX), cyw = 0.5 * (minY + maxY);
	const double zPad = std::max(1e-3, 0.05 * (maxZ - minZ));

	//ortho()'s near and far are distances along -z, so the visible band [minZ, maxZ] - in
	//which larger is NEARER - goes in as [-(maxZ + pad), -(minZ - pad)]
	QMatrix4x4 proj;
	proj.ortho((float)(cxw - halfW), (float)(cxw + halfW), (float)(cyw - halfH), (float)(cyw + halfH),
		(float)(-(maxZ + zPad)), (float)(-(minZ - zPad)));
	const QMatrix4x4 mvp = proj * rot;

	// ── draw ──
	uploadRamp(scene.ramp);
	m_fbo->bind();
	f->glViewport(0, 0, outSize.width(), outSize.height());
	f->glClearColor(kBgR, kBgG, kBgB, 1.0f);
	f->glClearDepth(1.0);
	f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	f->glEnable(GL_DEPTH_TEST);
	f->glDepthFunc(GL_LEQUAL);
	f->glDisable(GL_CULL_FACE);        //two-sided: a flank seen from behind is still the part
	f->glDisable(GL_BLEND);
	if (m_fbo->format().samples() > 0) f->glEnable(GL_MULTISAMPLE);

	f->glBindVertexArray(m_vao);
	f->glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
	f->glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(verts.size() * sizeof(Vtx)), verts.data(), GL_STREAM_DRAW);
	const std::vector<unsigned int>& elems = points ? pts : idx;
	f->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
	f->glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)(elems.size() * sizeof(unsigned int)),
		elems.data(), GL_STREAM_DRAW);

	m_prog->bind();
	m_prog->setUniformValue("uMVP", mvp);
	m_prog->setUniformValue("uNormal", rot.normalMatrix());
	m_prog->setUniformValue("uZSpan", (float)zSpan);
	//the CPU renderer measures its normal in (screen x, screen y DOWN, depth toward the
	//viewer); this matrix produces (screen x, screen y UP, depth toward the viewer), so only
	//the light's y component flips
	m_prog->setUniformValue("uLight", QVector3D(kLightX, -kLightY, kLightZ));
	m_prog->setUniformValue("uAmbient", kAmbient);
	m_prog->setUniformValue("uRamp", 0);
	m_prog->setUniformValue("uDarken", 1.0f);
	f->glActiveTexture(GL_TEXTURE0);
	f->glBindTexture(GL_TEXTURE_2D, m_rampTex);

	if (points) {
		//size the dot to the cell pitch, so the cloud reads as a surface when dense and as
		//separate samples when sparse - the same rule the CPU renderer uses
		const double pitch = s * dx;
		f->glPointSize((float)std::max(1.2, std::min(5.2, pitch * 1.24)));
		m_prog->setUniformValue("uMode", 0);
		m_prog->setUniformValue("uLit", 0);
		f->glDrawElements(GL_POINTS, (GLsizei)elems.size(), GL_UNSIGNED_INT, nullptr);
	}
	else if (wire) {
		/*
		* HIDDEN-LINE wireframe, not a see-through one: the facets are filled with the
		* background first so a nearer one erases the lines behind it, and only then are the
		* edges stroked. A transparent wireframe on a surface this dense collapses into noise.
		* The polygon offset pushes the fill away from the camera so it cannot z-fight the
		* lines drawn on top of it.
		*/
		m_prog->setUniformValue("uMode", 2);
		m_prog->setUniformValue("uLit", 0);
		m_prog->setUniformValue("uColor", QVector4D(kBgR, kBgG, kBgB, 1.0f));
		f->glEnable(GL_POLYGON_OFFSET_FILL);
		f->glPolygonOffset(1.0f, 1.0f);
		f->glDrawElements(GL_TRIANGLES, (GLsizei)elems.size(), GL_UNSIGNED_INT, nullptr);
		f->glDisable(GL_POLYGON_OFFSET_FILL);

		m_prog->setUniformValue("uMode", 0);
		f->glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		f->glDrawElements(GL_TRIANGLES, (GLsizei)elems.size(), GL_UNSIGNED_INT, nullptr);
		f->glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}
	else {
		m_prog->setUniformValue("uMode", textured ? 1 : 0);
		m_prog->setUniformValue("uLit", lit ? 1 : 0);
		f->glDrawElements(GL_TRIANGLES, (GLsizei)elems.size(), GL_UNSIGNED_INT, nullptr);

		if (meshLines) {
			//a darker edge of the facet's OWN colour, not a fixed black lattice: at a few pixels
			//per cell a black grid swamps the surface, while this reads as a mesh and still
			//carries the height colour
			m_prog->setUniformValue("uDarken", 0.60f);
			m_prog->setUniformValue("uLit", 0);
			f->glEnable(GL_POLYGON_OFFSET_LINE);
			f->glPolygonOffset(-1.0f, -1.0f);
			f->glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			f->glDrawElements(GL_TRIANGLES, (GLsizei)elems.size(), GL_UNSIGNED_INT, nullptr);
			f->glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			f->glDisable(GL_POLYGON_OFFSET_LINE);
			m_prog->setUniformValue("uDarken", 1.0f);
		}
	}

	m_prog->release();
	f->glBindVertexArray(0);
	m_fbo->release();
	//toImage() resolves the multisampled attachment and flips GL's bottom-left origin to the
	//top-left one a QImage expects
	QImage out = m_fbo->toImage().convertToFormat(QImage::Format_RGB888);
	m_ctx->doneCurrent();
	return out;
}

} //namespace

bool algoH3GLAvailable() { return GLBackend::instance().ready(); }

QString algoH3GLError() { return GLBackend::instance().error(); }

QImage algoH3RenderSurfaceGL(const AlgoH3GLScene& scene, double yawDeg, double pitchDeg,
	double zExaggeration, const QSize& outSize, AlgoH3SurfaceStyle style)
{
	if (!GLBackend::instance().ready()) return QImage();
	if (outSize.width() < 64 || outSize.height() < 64) return QImage();
	return GLBackend::instance().render(scene, yawDeg, pitchDeg, zExaggeration, outSize, style);
}
