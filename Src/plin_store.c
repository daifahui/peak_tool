#include <string.h>
#include "stm32f0xx_hal.h"
#include "plin_store.h"
#include "plin_protocol.h"

/* 调度表存储布局（2 个 2 KB 页，一页一张表）：
 *   0x0801F000 + n*0x800 : 表头（32 B）
 *   +32                  : 64 帧 × 16 B = 1024 B
 *   +1056                : 2 调度 × 8 槽 × 16 B = 256 B
 * 合计 1312 B < 2048 B，剩余空间留作后续扩展。 */

#define PLIN_STORE_BASE       0x0801F000u
#define PLIN_STORE_PAGE_SIZE  0x800u
#define PLIN_STORE_MAGIC      0x534E494Cu  /* "LINS" */
#define PLIN_STORE_VERSION    2u

#define STORE_FRAMES_SIZE ( PLIN_MAX_FRAMES * PLIN_FRAME_ENTRY_SIZE )
#define STORE_SLOTS_SIZE  ( PLIN_MAX_SCHEDULES * PLIN_MAX_SLOTS * PLIN_FRAME_ENTRY_SIZE )

struct plin_store_header
{
  uint32_t magic;
  uint16_t version;
  uint16_t baudrate;
  uint8_t  slot_count[PLIN_MAX_SCHEDULES];
  uint16_t frame_count;
  uint8_t  mode;
  uint8_t  autostart;
  uint8_t  active_schedule;
  uint8_t  valid[PLIN_MAX_FRAMES / 8];
  uint8_t  reserved[2];
  uint32_t crc;
};

typedef char plin_store_layout_check[
  ( sizeof( struct plin_store_header ) + STORE_FRAMES_SIZE + STORE_SLOTS_SIZE <= PLIN_STORE_PAGE_SIZE )
  ? 1 : -1 ];

/* 保存/加载共用的配置暂存（避免 1.2 KB 栈占用） */
static struct plin_config store_cfg;

static uint32_t store_addr( uint8_t table )
{
  return PLIN_STORE_BASE + (uint32_t)table * PLIN_STORE_PAGE_SIZE;
}

static const struct plin_store_header *store_header( uint8_t table )
{
  return (const struct plin_store_header*)store_addr( table );
}

/* ---- CRC32 ------------------------------------------------------------- */

static uint32_t crc32_begin( void )
{
  return 0xFFFFFFFFu;
}

static uint32_t crc32_update( uint32_t crc, const void *data, uint32_t length )
{
  const uint8_t *p = data;

  while( length-- )
  {
    crc ^= *p++;
    for( uint8_t i = 0; i < 8; i++ )
      crc = ( crc >> 1 ) ^ ( 0xEDB88320u & ( 0u - ( crc & 1u ) ) );
  }
  return crc;
}

static uint32_t crc32_end( uint32_t crc )
{
  return ~crc;
}

/* ---- Flash 驱动（直接寄存器操作，F0 页大小 2 KB） -------------------- */

static int flash_wait( void )
{
  while( FLASH->SR & FLASH_SR_BSY )
    ;

  if( FLASH->SR & ( FLASH_SR_PGERR | FLASH_SR_WRPERR ) )
  {
    FLASH->SR = FLASH_SR_PGERR | FLASH_SR_WRPERR;
    return -1;
  }

  FLASH->SR = FLASH_SR_EOP;
  return 0;
}

static void flash_unlock( void )
{
  if( FLASH->CR & FLASH_CR_LOCK )
  {
    FLASH->KEYR = 0x45670123u;
    FLASH->KEYR = 0xCDEF89ABu;
  }
}

static void flash_lock( void )
{
  FLASH->CR |= FLASH_CR_LOCK;
}

static int flash_erase_page( uint32_t addr )
{
  if( flash_wait() != 0 )
    return -1;

  FLASH->CR |= FLASH_CR_PER;
  FLASH->AR = addr;
  FLASH->CR |= FLASH_CR_STRT;

  if( flash_wait() != 0 )
  {
    FLASH->CR &= ~FLASH_CR_PER;
    return -1;
  }

  FLASH->CR &= ~FLASH_CR_PER;
  return 0;
}

static int flash_program_halfword( uint32_t addr, uint16_t value )
{
  if( flash_wait() != 0 )
    return -1;

  FLASH->CR |= FLASH_CR_PG;
  *(volatile uint16_t*)addr = value;

  if( flash_wait() != 0 )
  {
    FLASH->CR &= ~FLASH_CR_PG;
    return -1;
  }

  FLASH->CR &= ~FLASH_CR_PG;
  return 0;
}

static int flash_program( uint32_t addr, const void *data, uint32_t length )
{
  const uint8_t *p = data;

  for( uint32_t i = 0; i < length; i += 2 )
  {
    uint16_t halfword = (uint16_t)( p[i] | ( p[i + 1] << 8 ) );

    if( flash_program_halfword( addr + i, halfword ) != 0 )
      return -1;
  }
  return 0;
}

/* ---- 表校验 / 读写 ---------------------------------------------------- */

static int store_valid( uint8_t table )
{
  const struct plin_store_header *hdr;
  struct plin_store_header tmp;
  uint32_t crc;
  uint32_t calc;

  if( table >= PLIN_STORE_TABLES )
    return 0;

  hdr = store_header( table );
  if( hdr->magic != PLIN_STORE_MAGIC || hdr->version != PLIN_STORE_VERSION )
    return 0;

  tmp = *hdr;
  crc = tmp.crc;
  tmp.crc = 0;

  calc = crc32_update( crc32_begin(), &tmp, sizeof( tmp ) );
  calc = crc32_update( calc, (const void*)( store_addr( table ) + sizeof( tmp ) ),
                       STORE_FRAMES_SIZE + STORE_SLOTS_SIZE );

  return ( crc32_end( calc ) == crc ) ? 1 : 0;
}

int plin_store_save( uint8_t table, uint8_t autostart )
{
  struct plin_store_header hdr;
  uint32_t addr;
  uint32_t crc;
  uint16_t frame_count = 0;
  int res;

  if( table >= PLIN_STORE_TABLES )
    return -1;
  if( !plin_protocol_get_config( table, &store_cfg ) )
    return -1;

  memset( &hdr, 0, sizeof( hdr ) );
  hdr.magic = PLIN_STORE_MAGIC;
  hdr.version = PLIN_STORE_VERSION;
  hdr.baudrate = store_cfg.baudrate;
  memcpy( hdr.slot_count, store_cfg.slot_count, sizeof( hdr.slot_count ) );
  hdr.active_schedule = store_cfg.active_schedule;
  hdr.mode = store_cfg.mode;
  hdr.autostart = autostart ? 1 : 0;

  for( uint8_t i = 0; i < PLIN_MAX_FRAMES; i++ )
  {
    if( store_cfg.frame_valid[i] )
    {
      hdr.valid[i >> 3] |= (uint8_t)( 1u << ( i & 7 ) );
      frame_count++;
    }
  }
  hdr.frame_count = frame_count;

  crc = crc32_update( crc32_begin(), &hdr, sizeof( hdr ) );
  crc = crc32_update( crc, store_cfg.frames, sizeof( store_cfg.frames ) );
  crc = crc32_update( crc, store_cfg.slots, sizeof( store_cfg.slots ) );
  hdr.crc = crc32_end( crc );

  addr = store_addr( table );

  flash_unlock();
  res = flash_erase_page( addr );
  if( res == 0 )
    res = flash_program( addr, &hdr, sizeof( hdr ) );
  if( res == 0 )
    res = flash_program( addr + sizeof( hdr ), store_cfg.frames, sizeof( store_cfg.frames ) );
  if( res == 0 )
    res = flash_program( addr + sizeof( hdr ) + sizeof( store_cfg.frames ),
                         store_cfg.slots, sizeof( store_cfg.slots ) );
  flash_lock();

  return res;
}

int plin_store_load( uint8_t table, uint8_t start )
{
  const struct plin_store_header *hdr;

  if( !store_valid( table ) )
    return -1;

  hdr = store_header( table );

  memcpy( store_cfg.frames,
          (const void*)( store_addr( table ) + sizeof( *hdr ) ),
          sizeof( store_cfg.frames ) );
  memcpy( store_cfg.slots,
          (const void*)( store_addr( table ) + sizeof( *hdr ) + sizeof( store_cfg.frames ) ),
          sizeof( store_cfg.slots ) );

  memset( store_cfg.frame_valid, 0, sizeof( store_cfg.frame_valid ) );
  for( uint8_t i = 0; i < PLIN_MAX_FRAMES; i++ )
    store_cfg.frame_valid[i] = ( hdr->valid[i >> 3] & ( 1u << ( i & 7 ) ) ) ? 1 : 0;

  memcpy( store_cfg.slot_count, hdr->slot_count, sizeof( store_cfg.slot_count ) );
  store_cfg.active_schedule = hdr->active_schedule % PLIN_MAX_SCHEDULES;
  store_cfg.baudrate = hdr->baudrate;
  store_cfg.mode = hdr->mode;

  if( !plin_protocol_set_config( table, &store_cfg ) )
    return -1;

  if( start )
    plin_protocol_set_schedule_active( table, 1 );

  return 0;
}

void plin_store_init( void )
{
  for( uint8_t table = 0; table < PLIN_STORE_TABLES; table++ )
  {
    if( !store_valid( table ) )
      continue;

    (void)plin_store_load( table, store_header( table )->autostart ? 1 : 0 );
  }
}

void plin_store_info( uint8_t *info )
{
  if( info == 0 )
    return;

  memset( info, 0, 8 );

  for( uint8_t table = 0; table < PLIN_STORE_TABLES; table++ )
  {
    if( !store_valid( table ) )
      continue;

    {
      const struct plin_store_header *hdr = store_header( table );

      info[0 + table] = 1;
      info[2 + table] = hdr->autostart;
      info[4 + table] = hdr->mode;
      {
        uint8_t sched = (uint8_t)( hdr->active_schedule % PLIN_MAX_SCHEDULES );
        uint8_t count = hdr->slot_count[sched];

        info[6 + table] = ( count > PLIN_MAX_SLOTS ) ? PLIN_MAX_SLOTS : count;
      }
    }
  }
}
