// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: message/map/proto/map_rsu.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_message_2fmap_2fproto_2fmap_5frsu_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_message_2fmap_2fproto_2fmap_5frsu_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3014000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3014000 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include "message/map/proto/map_id.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_message_2fmap_2fproto_2fmap_5frsu_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_message_2fmap_2fproto_2fmap_5frsu_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxiliaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_message_2fmap_2fproto_2fmap_5frsu_2eproto;
namespace civ {
namespace hdmap {
class RSU;
class RSUDefaultTypeInternal;
extern RSUDefaultTypeInternal _RSU_default_instance_;
}  // namespace hdmap
}  // namespace civ
PROTOBUF_NAMESPACE_OPEN
template<> ::civ::hdmap::RSU* Arena::CreateMaybeMessage<::civ::hdmap::RSU>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace civ {
namespace hdmap {

// ===================================================================

class RSU PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:civ.hdmap.RSU) */ {
 public:
  inline RSU() : RSU(nullptr) {}
  virtual ~RSU();

  RSU(const RSU& from);
  RSU(RSU&& from) noexcept
    : RSU() {
    *this = ::std::move(from);
  }

  inline RSU& operator=(const RSU& from) {
    CopyFrom(from);
    return *this;
  }
  inline RSU& operator=(RSU&& from) noexcept {
    if (GetArena() == from.GetArena()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  inline const ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance);
  }
  inline ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return GetMetadataStatic().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return GetMetadataStatic().reflection;
  }
  static const RSU& default_instance();

  static inline const RSU* internal_default_instance() {
    return reinterpret_cast<const RSU*>(
               &_RSU_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(RSU& a, RSU& b) {
    a.Swap(&b);
  }
  inline void Swap(RSU* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(RSU* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline RSU* New() const final {
    return CreateMaybeMessage<RSU>(nullptr);
  }

  RSU* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<RSU>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const RSU& from);
  void MergeFrom(const RSU& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  inline void SharedCtor();
  inline void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(RSU* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "civ.hdmap.RSU";
  }
  protected:
  explicit RSU(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_message_2fmap_2fproto_2fmap_5frsu_2eproto);
    return ::descriptor_table_message_2fmap_2fproto_2fmap_5frsu_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kOverlapIdFieldNumber = 3,
    kIdFieldNumber = 1,
    kJunctionIdFieldNumber = 2,
  };
  // repeated .civ.hdmap.Id overlap_id = 3;
  int overlap_id_size() const;
  private:
  int _internal_overlap_id_size() const;
  public:
  void clear_overlap_id();
  ::civ::hdmap::Id* mutable_overlap_id(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::civ::hdmap::Id >*
      mutable_overlap_id();
  private:
  const ::civ::hdmap::Id& _internal_overlap_id(int index) const;
  ::civ::hdmap::Id* _internal_add_overlap_id();
  public:
  const ::civ::hdmap::Id& overlap_id(int index) const;
  ::civ::hdmap::Id* add_overlap_id();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::civ::hdmap::Id >&
      overlap_id() const;

  // optional .civ.hdmap.Id id = 1;
  bool has_id() const;
  private:
  bool _internal_has_id() const;
  public:
  void clear_id();
  const ::civ::hdmap::Id& id() const;
  ::civ::hdmap::Id* release_id();
  ::civ::hdmap::Id* mutable_id();
  void set_allocated_id(::civ::hdmap::Id* id);
  private:
  const ::civ::hdmap::Id& _internal_id() const;
  ::civ::hdmap::Id* _internal_mutable_id();
  public:
  void unsafe_arena_set_allocated_id(
      ::civ::hdmap::Id* id);
  ::civ::hdmap::Id* unsafe_arena_release_id();

  // optional .civ.hdmap.Id junction_id = 2;
  bool has_junction_id() const;
  private:
  bool _internal_has_junction_id() const;
  public:
  void clear_junction_id();
  const ::civ::hdmap::Id& junction_id() const;
  ::civ::hdmap::Id* release_junction_id();
  ::civ::hdmap::Id* mutable_junction_id();
  void set_allocated_junction_id(::civ::hdmap::Id* junction_id);
  private:
  const ::civ::hdmap::Id& _internal_junction_id() const;
  ::civ::hdmap::Id* _internal_mutable_junction_id();
  public:
  void unsafe_arena_set_allocated_junction_id(
      ::civ::hdmap::Id* junction_id);
  ::civ::hdmap::Id* unsafe_arena_release_junction_id();

  // @@protoc_insertion_point(class_scope:civ.hdmap.RSU)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::civ::hdmap::Id > overlap_id_;
  ::civ::hdmap::Id* id_;
  ::civ::hdmap::Id* junction_id_;
  friend struct ::TableStruct_message_2fmap_2fproto_2fmap_5frsu_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// RSU

// optional .civ.hdmap.Id id = 1;
inline bool RSU::_internal_has_id() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || id_ != nullptr);
  return value;
}
inline bool RSU::has_id() const {
  return _internal_has_id();
}
inline const ::civ::hdmap::Id& RSU::_internal_id() const {
  const ::civ::hdmap::Id* p = id_;
  return p != nullptr ? *p : reinterpret_cast<const ::civ::hdmap::Id&>(
      ::civ::hdmap::_Id_default_instance_);
}
inline const ::civ::hdmap::Id& RSU::id() const {
  // @@protoc_insertion_point(field_get:civ.hdmap.RSU.id)
  return _internal_id();
}
inline void RSU::unsafe_arena_set_allocated_id(
    ::civ::hdmap::Id* id) {
  if (GetArena() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(id_);
  }
  id_ = id;
  if (id) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:civ.hdmap.RSU.id)
}
inline ::civ::hdmap::Id* RSU::release_id() {
  _has_bits_[0] &= ~0x00000001u;
  ::civ::hdmap::Id* temp = id_;
  id_ = nullptr;
  if (GetArena() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
  return temp;
}
inline ::civ::hdmap::Id* RSU::unsafe_arena_release_id() {
  // @@protoc_insertion_point(field_release:civ.hdmap.RSU.id)
  _has_bits_[0] &= ~0x00000001u;
  ::civ::hdmap::Id* temp = id_;
  id_ = nullptr;
  return temp;
}
inline ::civ::hdmap::Id* RSU::_internal_mutable_id() {
  _has_bits_[0] |= 0x00000001u;
  if (id_ == nullptr) {
    auto* p = CreateMaybeMessage<::civ::hdmap::Id>(GetArena());
    id_ = p;
  }
  return id_;
}
inline ::civ::hdmap::Id* RSU::mutable_id() {
  // @@protoc_insertion_point(field_mutable:civ.hdmap.RSU.id)
  return _internal_mutable_id();
}
inline void RSU::set_allocated_id(::civ::hdmap::Id* id) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArena();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(id_);
  }
  if (id) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
      reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(id)->GetArena();
    if (message_arena != submessage_arena) {
      id = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, id, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  id_ = id;
  // @@protoc_insertion_point(field_set_allocated:civ.hdmap.RSU.id)
}

// optional .civ.hdmap.Id junction_id = 2;
inline bool RSU::_internal_has_junction_id() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  PROTOBUF_ASSUME(!value || junction_id_ != nullptr);
  return value;
}
inline bool RSU::has_junction_id() const {
  return _internal_has_junction_id();
}
inline const ::civ::hdmap::Id& RSU::_internal_junction_id() const {
  const ::civ::hdmap::Id* p = junction_id_;
  return p != nullptr ? *p : reinterpret_cast<const ::civ::hdmap::Id&>(
      ::civ::hdmap::_Id_default_instance_);
}
inline const ::civ::hdmap::Id& RSU::junction_id() const {
  // @@protoc_insertion_point(field_get:civ.hdmap.RSU.junction_id)
  return _internal_junction_id();
}
inline void RSU::unsafe_arena_set_allocated_junction_id(
    ::civ::hdmap::Id* junction_id) {
  if (GetArena() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(junction_id_);
  }
  junction_id_ = junction_id;
  if (junction_id) {
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:civ.hdmap.RSU.junction_id)
}
inline ::civ::hdmap::Id* RSU::release_junction_id() {
  _has_bits_[0] &= ~0x00000002u;
  ::civ::hdmap::Id* temp = junction_id_;
  junction_id_ = nullptr;
  if (GetArena() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
  return temp;
}
inline ::civ::hdmap::Id* RSU::unsafe_arena_release_junction_id() {
  // @@protoc_insertion_point(field_release:civ.hdmap.RSU.junction_id)
  _has_bits_[0] &= ~0x00000002u;
  ::civ::hdmap::Id* temp = junction_id_;
  junction_id_ = nullptr;
  return temp;
}
inline ::civ::hdmap::Id* RSU::_internal_mutable_junction_id() {
  _has_bits_[0] |= 0x00000002u;
  if (junction_id_ == nullptr) {
    auto* p = CreateMaybeMessage<::civ::hdmap::Id>(GetArena());
    junction_id_ = p;
  }
  return junction_id_;
}
inline ::civ::hdmap::Id* RSU::mutable_junction_id() {
  // @@protoc_insertion_point(field_mutable:civ.hdmap.RSU.junction_id)
  return _internal_mutable_junction_id();
}
inline void RSU::set_allocated_junction_id(::civ::hdmap::Id* junction_id) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArena();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(junction_id_);
  }
  if (junction_id) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
      reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(junction_id)->GetArena();
    if (message_arena != submessage_arena) {
      junction_id = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, junction_id, submessage_arena);
    }
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  junction_id_ = junction_id;
  // @@protoc_insertion_point(field_set_allocated:civ.hdmap.RSU.junction_id)
}

// repeated .civ.hdmap.Id overlap_id = 3;
inline int RSU::_internal_overlap_id_size() const {
  return overlap_id_.size();
}
inline int RSU::overlap_id_size() const {
  return _internal_overlap_id_size();
}
inline ::civ::hdmap::Id* RSU::mutable_overlap_id(int index) {
  // @@protoc_insertion_point(field_mutable:civ.hdmap.RSU.overlap_id)
  return overlap_id_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::civ::hdmap::Id >*
RSU::mutable_overlap_id() {
  // @@protoc_insertion_point(field_mutable_list:civ.hdmap.RSU.overlap_id)
  return &overlap_id_;
}
inline const ::civ::hdmap::Id& RSU::_internal_overlap_id(int index) const {
  return overlap_id_.Get(index);
}
inline const ::civ::hdmap::Id& RSU::overlap_id(int index) const {
  // @@protoc_insertion_point(field_get:civ.hdmap.RSU.overlap_id)
  return _internal_overlap_id(index);
}
inline ::civ::hdmap::Id* RSU::_internal_add_overlap_id() {
  return overlap_id_.Add();
}
inline ::civ::hdmap::Id* RSU::add_overlap_id() {
  // @@protoc_insertion_point(field_add:civ.hdmap.RSU.overlap_id)
  return _internal_add_overlap_id();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::civ::hdmap::Id >&
RSU::overlap_id() const {
  // @@protoc_insertion_point(field_list:civ.hdmap.RSU.overlap_id)
  return overlap_id_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace hdmap
}  // namespace civ

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_message_2fmap_2fproto_2fmap_5frsu_2eproto
