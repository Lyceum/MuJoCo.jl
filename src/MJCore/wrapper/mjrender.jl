const NAUX = 10
const MAXTEXTURE = 1000

@enum mjtGridPos::Cint begin
    GRID_TOPLEFT
    GRID_TOPRIGHT
    GRID_BOTTOMLEFT
    GRID_BOTTOMRIGHT
end

@enum mjtFramebuffer::Cint begin
    FB_WINDOW
    FB_OFFSCREEN
end

@enum mjtFontScale::Cint begin
    FONTSCALE_50 = 50
    FONTSCALE_100 = 100
    FONTSCALE_150 = 150
    FONTSCALE_200 = 200
    FONTSCALE_250 = 250
    FONTSCALE_300 = 300
end

@enum mjtFont::Cint begin
    FONT_NORMAL
    FONT_SHADOW
    FONT_BIG
end

@uninitable struct mjrRect
    left::Cint
    bottom::Cint
    width::Cint
    height::Cint
end

@uninitable mutable struct mjrContext
    lineWidth::Cfloat
    shadowClip::Cfloat
    shadowScale::Cfloat
    fogStart::Cfloat
    fogEnd::Cfloat
    fogRGBA::SVector{4,Cfloat}
    shadowSize::Cint
    offWidth::Cint
    offHeight::Cint
    offSamples::Cint
    fontScale::Cint
    auxWidth::SVector{NAUX,Cint}
    auxHeight::SVector{NAUX,Cint}
    auxSamples::SVector{NAUX,Cint}

    offFBO::UInt32
    offFBO_r::UInt32
    offColor::UInt32
    offColor_r::UInt32
    offDepthStencil::UInt32
    offDepthStencil_r::UInt32
    shadowFBO::UInt32
    shadowTex::UInt32

    auxFBO::SVector{NAUX,UInt32}
    auxFBO_r::SVector{NAUX,UInt32}
    auxColor::SVector{NAUX,UInt32}
    auxColor_r::SVector{NAUX,UInt32}

    ntexture::Cint
    textureType::SVector{100,Cint}
    texture::SVector{100,UInt32}
    basePlane::UInt32
    baseMesh::UInt32
    baseHField::UInt32
    baseBuiltin::UInt32
    baseFontNormal::UInt32
    baseFontShadow::UInt32
    baseFontBig::UInt32
    rangePlane::Cint
    rangeMesh::Cint
    rangeHField::Cint
    rangeBuiltin::Cint
    rangeFont::Cint

    nskin::Cint
    skinvertVBO::Ptr{UInt32}
    skinnormalVBO::Ptr{UInt32}
    skintexcoordVBO::Ptr{UInt32}
    skinfaceVBO::Ptr{UInt32}

    charWidth::SVector{127,Cint}
    charWidthBig::SVector{127,Cint}
    charHeight::Cint
    charHeightBig::Cint

    glewInitialized::Cint
    windowAvailable::Cint
    windowSamples::Cint
    windowStereo::Cint
    windowDoublebuffer::Cint
    currentBuffer::Cint
end

